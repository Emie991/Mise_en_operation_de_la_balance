
// #define _DEFAULT_SOURCE
// #define _GNU_SOURCE

// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <string.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <sys/wait.h>
// #include <signal.h>
// #include <sys/ioctl.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <termios.h>
// #include <net/if.h>  // Pour struct ifreq

// //#define PILOTESERIEUSB_TTY "/dev/ttyUSB0"
// #define PILOTESERIEUSB_TTY "/dev/ttyUSB1"

// // Variables globales pour la communication série et CAN
// int serial_fd, can_socket;
// int gram_mode = 1;  // 1 pour grammes, 0 pour onces

// // Fonction d'initialisation de la balance
// int init_balance() {
//     struct termios options;
//     serial_fd = open(PILOTESERIEUSB_TTY, O_RDWR | O_NOCTTY);
//     if (serial_fd == -1) {
//         perror("Erreur d'ouverture de la balance");
//         return -1;
//     }
//     tcgetattr(serial_fd, &options);
//     cfsetispeed(&options, B9600);
//     cfsetospeed(&options, B9600);
//     options.c_cflag |= (CLOCAL | CREAD);
//     tcsetattr(serial_fd, TCSANOW, &options);
//     return 0;
// }

// // Fonction d'initialisation du CAN
// int init_can() {
//     struct sockaddr_can addr;
//     struct ifreq ifr;

//     if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
//         perror("Erreur lors de la création du socket CAN");
//         return -1;
//     }

//     strcpy(ifr.ifr_name, "vcan0");
//     ioctl(can_socket, SIOCGIFINDEX, &ifr);
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;

//     if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//         perror("Erreur de liaison du socket CAN");
//         return -1;
//     }

//     return 0;
// }

// // Fonction pour lire le poids de la balance
// float lire_poids_balance() 
// {
//   char buffer[32];
//     char filtered_buffer[32];
//     int j = 0;

//     int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
//     if (bytes_read > 0) {
//         buffer[bytes_read] = '\0';

//         // Filtrer pour ne garder que les chiffres et les points
//         for (int i = 0; i < bytes_read; i++) {
//             if ((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.') 
//             {
//                 filtered_buffer[j++] = buffer[i];
//             }
//         }
//         filtered_buffer[j] = '\0';


//         // Conversion en float et retour
//         return atof(filtered_buffer);
//     }
//     return -1;
// }


// // Processus enfant pour gérer les messages CAN
// void process_can() 
// {
//     struct can_frame frame;
//     while (1) 
//     {
//         if (read(can_socket, &frame, sizeof(struct can_frame)) > 0) 
//         {
//             // Décoder les messages CAN ici
//             if (frame.can_id == 0x123) 
//             { // Adresse à définir
//                 gram_mode = frame.data[0] == 1;
//             } else if (frame.can_id == 0x124) {
//                 printf("CAN: Mode de fonctionnement %d\n", frame.data[0]);
//             }
//         }
//     }
// }


// void main_process() 
// {
//     float dernier_poids = 0.0;

//     while (1) 
//     {
//         float poids = lire_poids_balance();
//         if (poids >= 0)
//         {
//             // Conversion si le mode est en onces
//             if (!gram_mode) 
//             {
//                 poids *= 0.035274;
//             }

//             dernier_poids = poids; // Mettre à jour la dernière valeur valide

//             // Affichage du poids dans le terminal
//             if(poids > 60 && poids < 100 )
//             {
//             printf("Poids: %.2f %s\n", poids, gram_mode ? "g" : "oz");

//             // Envoi des données CAN
//             struct can_frame frame;
//             frame.can_id = 0x125; // Adresse à définir
//             frame.can_dlc = sizeof(float);
//             memcpy(frame.data, &poids, sizeof(float));
//             write(can_socket, &frame, sizeof(struct can_frame));
//             }
//         }
//         else
//         {
//             // Réafficher la dernière valeur valide si aucune nouvelle donnée n'est disponible
//             printf("Poids: %.2f %s (dernier poids valide)\n", dernier_poids, gram_mode ? "g" : "oz");
//         }
//         sleep(1);
//     }
// }

// int main() {
//     // Initialisation
//     if (init_balance() == -1 || init_can() == -1) {
//         return 1;
//     }

//      // Création des processus
//     pid_t pid = fork();
//     if (pid == 0) {
//         // Processus enfant pour gérer le CAN
//         process_can();
//     } else if (pid > 0) {
//         // Processus parent pour gérer la balance
//         main_process();
//         wait(NULL); // Attendre le processus enfant
//     } else {
//         perror("Erreur de création de processus");
//     }

//     // Nettoyage
//     close(serial_fd);
//     close(can_socket);
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
#include <net/if.h>

//#define PILOTESERIEUSB_TTY "/dev/ttyUSB0"
#define PILOTESERIEUSB_TTY "/dev/ttyUSB1"

// Variables globales pour la communication série et CAN
int serial_fd, can_socket;
int gram_mode = 1;  // 1 pour grammes, 0 pour onces
int operation_mode = 0; // 0 pour arrêt, 1 pour opération
typedef enum
{
  Commandant = 0x120,
    com_mode,
    com_alarm,
  
  Station_Pese = 0x140,
    bal_mode,
    bal_poids,
    bal_poids_conversion,
  
  
  Centre_tri = 0x150,
    ct_couleur,
    ct_mode,
  
  Gestion_transport = 0x160,
    gt_position,
   gt_statut,
	
};

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
float lire_poids_balance() {
    char buffer[32];
    char filtered_buffer[32];
    int j = 0;

    int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';

        for (int i = 0; i < bytes_read; i++) {
            if ((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.') {
                filtered_buffer[j++] = buffer[i];
            }
        }
        filtered_buffer[j] = '\0';

        return atof(filtered_buffer);
    }
    return -1;
}

// Processus enfant pour gérer les messages CAN
void process_can() {
    struct can_frame frame;
    while (1) {
        if (read(can_socket, &frame, sizeof(struct can_frame)) > 0) {
            if (frame.can_id == bal_mode) {
                if (strncmp((char *)frame.data, "$Start\n", frame.can_dlc) == 0) {
                    operation_mode = 1;
                } 
else if (strncmp((char *)frame.data, "$Arret\n", frame.can_dlc) == 0) 
{
                    operation_mode = 0;
                }
            } else if (frame.can_id == bal_poids_conversion) {
                if (strncmp((char *)frame.data, "$Grammes\n", frame.can_dlc) == 0) {
                    gram_mode = 1;
                } else if (strncmp((char *)frame.data, "$Onces\n", frame.can_dlc) == 0) {
                    gram_mode = 0;
                }
            }
        }
    }
}

// Processus principal
void main_process() {
    float dernier_poids = 0.0;

    while (1) {
        if (operation_mode == 1) { // Mode opération
            float poids = lire_poids_balance();
            if (poids >= 0) {
                if (!gram_mode) {
                    poids *= 0.035274; // Conversion en onces
                }

                dernier_poids = poids;

                printf("Poids: %.2f %s\n", poids, gram_mode ? "g" : "oz");

                struct can_frame frame;
                frame.can_id = bal_poids;
                frame.can_dlc = sizeof(float);
                memcpy(frame.data, &poids, sizeof(float));
                write(can_socket, &frame, sizeof(struct can_frame));
            } else {
                printf("Poids: %.2f %s (dernier poids valide)\n", dernier_poids, gram_mode ? "g" : "oz");
            }
        } else {
            printf("Mode arrêt. En attente...\n");
        }
        sleep(1);
    }
}

int main() {
    if (init_balance() == -1 || init_can() == -1) {
        return 1;
    }

    pid_t pid = fork();
    if (pid == 0) {
        process_can();
    } else if (pid > 0) {
        main_process();
        wait(NULL);
    } else {
        perror("Erreur de création de processus");
    }

    close(serial_fd);
    close(can_socket);
    return 0;
}


