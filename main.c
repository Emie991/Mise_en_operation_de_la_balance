// #define _DEFAULT_SOURCE
// #define _GNU_SOURCE

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <errno.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <net/if.h>
// #include <termios.h>

// #define PILOTESERIEUSB_TTY "/dev/ttyUSB0"
// #define CAN_INTERFACE "vcan0"

// // États de la station
// typedef enum {
//     MODE_ARRET,
//     MODE_OPERATION,
//     MODE_ERREUR
// } EtatStation;

// EtatStation etat_station = MODE_ARRET;

// // Fonctions pour la communication série
// int ouvrir_port_serie(const char* port) {
//     int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0) {
//         perror("Erreur d'ouverture du port série");
//         return -1;
//     }
//     struct termios tty;
//     memset(&tty, 0, sizeof tty);
//     if (tcgetattr(fd, &tty) != 0) {
//         perror("Erreur tcgetattr");
//         return -1;
//     }
//     cfsetospeed(&tty, B115200);
//     cfsetispeed(&tty, B115200);
//     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
//     tty.c_iflag &= ~IGNBRK;
//     tty.c_lflag = 0;
//     tty.c_oflag = 0;
//     tty.c_cc[VMIN] = 1;
//     tty.c_cc[VTIME] = 5;
//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);
//     tty.c_cflag |= (CLOCAL | CREAD);
//     tty.c_cflag &= ~(PARENB | PARODD);
//     tty.c_cflag &= ~CSTOPB;
//     tty.c_cflag &= ~CRTSCTS;
//     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
//         perror("Erreur tcsetattr");
//         return -1;
//     }
//     return fd;
// }

// // Fonction pour lire le poids de la balance
// void lire_poids_balance(int fd) {
//     char buffer[256];
//     int n = read(fd, buffer, sizeof(buffer) - 1);
//     if (n > 0) {
//         buffer[n] = '\0';
//         printf("Poids: %s\n", buffer);
//         // Traitez le poids ici
//     }
// }

// // Fonction pour la gestion des messages CAN
// int configurer_can_socket(const char* interface) {
//     int socket_can;
//     struct sockaddr_can addr;
//     struct ifreq ifr;

//     socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (socket_can < 0) {
//         perror("Erreur d'ouverture du socket CAN");
//         return -1;
//     }
//     strcpy(ifr.ifr_name, interface);
//     ioctl(socket_can, SIOCGIFINDEX, &ifr);
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;

//     if (bind(socket_can, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//         perror("Erreur de liaison du socket CAN");
//         return -1;
//     }
//     return socket_can;
// }

// // Fonction de traitement des messages CAN
// void traiter_message_can(struct can_frame frame) {
//     switch (frame.can_id) {
//         case 0x100:  // ID de message pour le mode arrêt
//             etat_station = MODE_ARRET;
//             break;
//         case 0x101:  // ID de message pour le mode opération
//             etat_station = MODE_OPERATION;
//             break;
//         case 0x102:  // ID de message pour les unités (grammes/onces)
//             // Traiter l'unité de poids ici
//             break;
//         default:
//             printf("Message CAN inconnu\n");
//             break;
//     }
// }

// // Émission d'un message UART formaté
// void emettre_message_uart(int fd, const char* couleur, int poids, int position, EtatStation mode) {
//     char buffer[256];
//     snprintf(buffer, sizeof(buffer), "%s,%d,%d,%d\n", couleur, poids, position, mode);
//     write(fd, buffer, strlen(buffer));
// }

// // Boucle principale de la station
// void boucle_station(int fd_serie, int fd_can) {
//     struct can_frame frame;
//     while (1) {
//         // Lire le poids de la balance si en mode opération
//         if (etat_station == MODE_OPERATION) {
//             lire_poids_balance(fd_serie);
//             // Envoyer les informations par UART
//             emettre_message_uart(fd_serie, "bleu", 100, 1, etat_station);
//         }

//         // Recevoir et traiter un message CAN
//         int nbytes = read(fd_can, &frame, sizeof(struct can_frame));
//         if (nbytes > 0) {
//             traiter_message_can(frame);
//         }

//         // Émission d'état si besoin
//         // Peut inclure des états supplémentaires (ex: erreur)
//     }
// }

// int main() {
//     int fd_serie = ouvrir_port_serie(PILOTESERIEUSB_TTY);
//     if (fd_serie < 0) return -1;

//     int fd_can = configurer_can_socket(CAN_INTERFACE);
//     if (fd_can < 0) return -1;

//     boucle_station(fd_serie, fd_can);

//     close(fd_serie);
//     close(fd_can);
//     return 0;
// }

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
#include <net/if.h>  // Pour struct ifreq

//#define PILOTESERIEUSB_TTY "/dev/ttyUSB0"
#define PILOTESERIEUSB_TTY "/dev/ttyUSB1"

// Variables globales pour la communication série et CAN
int serial_fd, can_socket;
int gram_mode = 1;  // 1 pour grammes, 0 pour onces

// Fonction d'initialisation de la balance
int init_balance() {
    struct termios options;
    serial_fd = open(PILOTESERIEUSB_TTY, O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        perror("Erreur d'ouverture de la balance");
        return -1;
    }
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_fd, TCSANOW, &options);
    return 0;
}

// Fonction d'initialisation du CAN
int init_can() {
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Erreur lors de la création du socket CAN");
        return -1;
    }

    strcpy(ifr.ifr_name, "vcan0");
    ioctl(can_socket, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Erreur de liaison du socket CAN");
        return -1;
    }

    return 0;
}

// Fonction pour lire le poids de la balance
float lire_poids_balance() 
{
  char buffer[32];
    char filtered_buffer[32];
    int j = 0;

    int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';

        // Filtrer pour ne garder que les chiffres et les points
        for (int i = 0; i < bytes_read; i++) {
            if ((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.') 
            {
                filtered_buffer[j++] = buffer[i];
            }
        }
        filtered_buffer[j] = '\0';


        // Conversion en float et retour
        return atof(filtered_buffer);
    }
    return -1;
}


// Processus enfant pour gérer les messages CAN
void process_can() {
    struct can_frame frame;
    while (1) {
        if (read(can_socket, &frame, sizeof(struct can_frame)) > 0) {
            // Décoder les messages CAN ici
            if (frame.can_id == 0x123) { // Adresse à définir
                gram_mode = frame.data[0] == 1;
            } else if (frame.can_id == 0x124) {
                printf("CAN: Mode de fonctionnement %d\n", frame.data[0]);
            }
        }
    }
}


void main_process() 
{
    float dernier_poids = 0.0;

    while (1) 
    {
        float poids = lire_poids_balance();
        if (poids >= 0)
        {
            // Conversion si le mode est en onces
            if (!gram_mode) 
            {
                poids *= 0.035274;
            }

            dernier_poids = poids; // Mettre à jour la dernière valeur valide

            // Affichage du poids dans le terminal
            if(poids > 60 && poids < 100 )
            {
            printf("Poids: %.2f %s\n", poids, gram_mode ? "g" : "oz");

            // Envoi des données CAN
            struct can_frame frame;
            frame.can_id = 0x125; // Adresse à définir
            frame.can_dlc = sizeof(float);
            memcpy(frame.data, &poids, sizeof(float));
            write(can_socket, &frame, sizeof(struct can_frame));
            }
        }
        else
        {
            // Réafficher la dernière valeur valide si aucune nouvelle donnée n'est disponible
            printf("Poids: %.2f %s (dernier poids valide)\n", dernier_poids, gram_mode ? "g" : "oz");
        }
        sleep(1);
    }
}

int main() {
    // Initialisation
    if (init_balance() == -1 || init_can() == -1) {
        return 1;
    }

     // Création des processus
    pid_t pid = fork();
    if (pid == 0) {
        // Processus enfant pour gérer le CAN
        process_can();
    } else if (pid > 0) {
        // Processus parent pour gérer la balance
        main_process();
        wait(NULL); // Attendre le processus enfant
    } else {
        perror("Erreur de création de processus");
    }

    // Nettoyage
    close(serial_fd);
    close(can_socket);
    return 0;
}



