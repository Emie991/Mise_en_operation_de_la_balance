#define _DEFAULT_SOURCE
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <termios.h>

#define PILOTESERIEUSB_TTY "/dev/ttyUSB0"
#define CAN_INTERFACE "vcan0"

// États de la station
typedef enum {
    MODE_ARRET,
    MODE_OPERATION,
    MODE_ERREUR
} EtatStation;

EtatStation etat_station = MODE_ARRET;

// Fonctions pour la communication série
int ouvrir_port_serie(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erreur d'ouverture du port série");
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("Erreur tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Erreur tcsetattr");
        return -1;
    }
    return fd;
}

// Fonction pour lire le poids de la balance
void lire_poids_balance(int fd) {
    char buffer[256];
    int n = read(fd, buffer, sizeof(buffer) - 1);
    if (n > 0) {
        buffer[n] = '\0';
        printf("Poids: %s\n", buffer);
        // Traitez le poids ici
    }
}

// Fonction pour la gestion des messages CAN
int configurer_can_socket(const char* interface) {
    int socket_can;
    struct sockaddr_can addr;
    struct ifreq ifr;

    socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_can < 0) {
        perror("Erreur d'ouverture du socket CAN");
        return -1;
    }
    strcpy(ifr.ifr_name, interface);
    ioctl(socket_can, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_can, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Erreur de liaison du socket CAN");
        return -1;
    }
    return socket_can;
}

// Fonction de traitement des messages CAN
void traiter_message_can(struct can_frame frame) {
    switch (frame.can_id) {
        case 0x100:  // ID de message pour le mode arrêt
            etat_station = MODE_ARRET;
            break;
        case 0x101:  // ID de message pour le mode opération
            etat_station = MODE_OPERATION;
            break;
        case 0x102:  // ID de message pour les unités (grammes/onces)
            // Traiter l'unité de poids ici
            break;
        default:
            printf("Message CAN inconnu\n");
            break;
    }
}

// Émission d'un message UART formaté
void emettre_message_uart(int fd, const char* couleur, int poids, int position, EtatStation mode) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s,%d,%d,%d\n", couleur, poids, position, mode);
    write(fd, buffer, strlen(buffer));
}

// Boucle principale de la station
void boucle_station(int fd_serie, int fd_can) {
    struct can_frame frame;
    while (1) {
        // Lire le poids de la balance si en mode opération
        if (etat_station == MODE_OPERATION) {
            lire_poids_balance(fd_serie);
            // Envoyer les informations par UART
            emettre_message_uart(fd_serie, "bleu", 100, 1, etat_station);
        }

        // Recevoir et traiter un message CAN
        int nbytes = read(fd_can, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            traiter_message_can(frame);
        }

        // Émission d'état si besoin
        // Peut inclure des états supplémentaires (ex: erreur)
    }
}

int main() {
    int fd_serie = ouvrir_port_serie(PILOTESERIEUSB_TTY);
    if (fd_serie < 0) return -1;

    int fd_can = configurer_can_socket(CAN_INTERFACE);
    if (fd_can < 0) return -1;

    boucle_station(fd_serie, fd_can);

    close(fd_serie);
    close(fd_can);
    return 0;
}
