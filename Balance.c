/**
 * @file    station_pesage.c
 * 
 * @brief   Mise en opération d'une station de pesage connectée. 
 *          Ce programme permet la communication avec une balance via une passerelle CAN/série
 *          et la gestion des modes de la station (arrêt ou opération).
 *          La station lit le poids depuis une balance et peut transmettre les données via un réseau CAN.
 * 
 * @details
 *          - Utilisation du port série pour communiquer avec la balance.
 *          - Utilisation d'un réseau CAN pour transmettre les données.
 *          - Conversion des valeurs de poids en grammes ou en onces.
 *          - Gestion des commandes reçues via CAN pour changer les modes de fonctionnement.
 * 
 * @author  Émie-Jeanne Dupuis
 * @date    2024-12-01
 */

#define _DEFAULT_SOURCE 
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <net/if.h>

//#define PILOTESERIEUSB_TTY "/dev/ttyUSB0"
#define PILOTESERIEUSB_TTY "/dev/ttyUSB1"

// Variables globales pour la communication série et CAN
int serial_fd, can_socket;
int gram_mode = 1;  // 1 pour grammes, 0 pour onces
int operation_mode = 0; // 0 pour arrêt, 1 pour opération

// Enumération pour identifier les différents types de messages CAN
typedef enum
{
  Commandant = 0x120,
    com_mode,
    com_alarm,
    com_conversion, 
  
  Station_Pese = 0x140,
    bal_mode,
    bal_poids,
  
  Centre_tri = 0x150,
    ct_couleur,
    ct_mode,
  
  Gestion_transport = 0x160,
    gt_position,
   gt_statut,
	
}CAN_ID;

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Initialise et configure la communication série avec la balance.
//  Paramètres d'entrées       : Aucun
//  Paramètres de sortie       : 0 en cas de succès, -1 en cas d'erreur
//  Notes                      : Configure les options série (vitesse, parité, etc.).
// *************************************************************************************************
int init_balance() 
{
    struct termios options;

    // Ouverture du port série
    serial_fd = open(PILOTESERIEUSB_TTY, O_RDWR | O_NOCTTY);
    if (serial_fd == -1)
    {
        perror("Erreur d'ouverture de la balance");
        return -1;
    }

    // Configuration de la vitesse de communication à 9600 baud
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_fd, TCSANOW, &options);
    return 0;
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Initialise et configure le socket CAN pour la communication CAN.
//  Paramètres d'entrées       : Aucun
//  Paramètres de sortie       : 0 en cas de succès, -1 en cas d'erreur
//  Notes                      : Configure l'interface CAN (vcan0).
// *************************************************************************************************
int init_can() 
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    
     // Création d'un socket CAN 
    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
    {
        perror("Erreur lors de la création du socket CAN");
        return -1;
    }

    // Liaison du socket à l'interface réseau "can0"
    strcpy(ifr.ifr_name, "can0");
    ioctl(can_socket, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
    {
        perror("Erreur de liaison du socket CAN");
        return -1;
    }

    return 0;
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Lit le poids mesuré par la balance via la communication série.
//  Paramètres d'entrées       : Aucun
//  Paramètres de sortie       : Poids mesuré en float, -1 en cas d'erreur
//  Notes                      : Filtre les caractères non numériques pour extraire la valeur du poids.
// *************************************************************************************************
float lire_poids_balance() 
{
    char buffer[32];
    char filtered_buffer[32];
    int j = 0;
    
    // Lecture des données de la balance
    int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0)
    {
        buffer[bytes_read] = '\0';
        
        // Filtrage pour ne garder que les chiffres et le point
        for (int i = 0; i < bytes_read; i++) 
        {
            if ((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.') 
            {
                filtered_buffer[j++] = buffer[i];
            }
        }
        filtered_buffer[j] = '\0';

        // Conversion des données en nombre flottant
        return atof(filtered_buffer);
    }
    return -1;
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Gère les messages CAN reçus et ajuste les modes de fonctionnement.
//  Paramètres d'entrées       : Aucun
//  Paramètres de sortie       : Aucun
//  Notes                      : Cette fonction est exécutée par un processus enfant.
// *************************************************************************************************
void process_can() 
{
    struct can_frame frame;
    while (1) 
    {   
        // Lecture des messages CAN reçus
        if (read(can_socket, &frame, sizeof(struct can_frame)) > 0) 
        {
            if (frame.can_id == com_mode)
            {
                if (strncmp((char *)frame.data, "$Start\n", frame.can_dlc) == 0) 
                {
                    operation_mode = 1;
                } 
                else if (strncmp((char *)frame.data, "$Arret\n", frame.can_dlc) == 0) 
                {
                    operation_mode = 0;
                }

            }
            else if (frame.can_id == com_conversion) 
            {
                if (strncmp((char *)frame.data, "$Grammes\n", frame.can_dlc) == 0) 
                {
                    gram_mode = 1;
                } 
                else if (strncmp((char *)frame.data, "$Onces\n", frame.can_dlc) == 0) 
                {
                    gram_mode = 0;
                }
            }
        }
    }
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Processus principal qui lit le poids et envoie les données via CAN.
//  Paramètres d'entrées       : Aucun
//  Paramètres de sortie       : Aucun
//  Notes                      : Effectue la conversion du poids si nécessaire et affiche les résultats.
// *************************************************************************************************
void main_process() 
{
    float dernier_poids = 0.0;

    while (1) 
    {
        if (operation_mode == 1) // Si le mode opération est activé
        { 
            float poids = lire_poids_balance();
            if (poids >= 0) 
            {
                if (!gram_mode) 
                {
                    poids *= 0.035274; // Conversion en onces
                }

                dernier_poids = poids;

                printf("Poids: %.2f %s\n", poids, gram_mode ? "g" : "oz");

                // Envoi du poids via CAN
                struct can_frame frame;
                frame.can_id = bal_poids;
                frame.can_dlc = sizeof(float);
                memcpy(frame.data, &poids, sizeof(float));
                write(can_socket, &frame, sizeof(struct can_frame));
            } 
            else 
            {
                printf("Poids: %.2f %s (dernier poids valide)\n", dernier_poids, gram_mode ? "g" : "oz");
            }
        } 
        else 
        {
            printf("Mode arrêt. En attente...\n");
        }
        sleep(1);
    }
}

// *************************************************************************************************
//  Auteur                     : Émie-Jeanne Dupuis
//  Description                : Point d'entrée principal du programme.
//  Paramètres d'entrées       : Aucun
//  Paramètres de sortie       : 0 en cas de succès, 1 en cas d'erreur
//  Notes                      : Initialise les composants et lance les processus enfant et principal.
// *************************************************************************************************
int main() 
{
    if (init_balance() == -1 || init_can() == -1)
    {
        return 1;
    }

    // Création d'un processus enfant pour gérer les messages CAN
    pid_t pid = fork();
    if (pid == 0) 
    {
        process_can();
    } 
    else if (pid > 0) 
    {
        main_process();
        wait(NULL); // Attente de la fin du processus enfant
    } 
    else 
    {
        perror("Erreur de création de processus");
    }

    // Fermeture des périphériques
    close(serial_fd);
    close(can_socket);
    return 0;
}


