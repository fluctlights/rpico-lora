#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 12000
#define BUFFER_SIZE 1024

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len;
    char buffer[BUFFER_SIZE];

    // Crear el socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Error al crear el socket");
        exit(EXIT_FAILURE);
    }

    // Configurar la dirección del servidor
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // Escuchar en todas las interfaces
    server_addr.sin_port = htons(PORT);      // Puerto 12000

    // Asociar el socket con la dirección y puerto
    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("Error en bind");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // Escuchar conexiones entrantes
    if (listen(server_fd, 5) == -1) {
        perror("Error en listen");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("Servidor TCP escuchando en el puerto %d\n", PORT);

    while (1) {
        client_len = sizeof(client_addr);

        // Aceptar una nueva conexión
        if ((client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len)) == -1) {
            perror("Error en accept");
            continue;
        }

        printf("Conexión aceptada de %s:%d\n",
               inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // Leer datos del cliente
        ssize_t bytes_received;
        while ((bytes_received = recv(client_fd, buffer, BUFFER_SIZE - 1, 0)) > 0) {
            buffer[bytes_received] = '\0'; // Asegurarse de que la cadena esté terminada
            printf("Mensaje recibido: %s\n", buffer);
        }

        if (bytes_received == 0) {
            printf("Cliente desconectado: %s:%d\n",
                   inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        } else if (bytes_received == -1) {
            perror("Error al recibir datos");
        }

        // Cerrar la conexión con el cliente
        close(client_fd);
    }

    // Cerrar el socket del servidor
    close(server_fd);
    return 0;
}
