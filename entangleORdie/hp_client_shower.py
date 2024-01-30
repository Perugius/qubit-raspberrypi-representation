#run on computer to show hp

import flet as ft
import time
import socket

# Define the IP address and port to listen on
computer_ip = "192.168.178.20"
computer_port = 5005

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the address
sock.bind((computer_ip, computer_port))

def main(page: ft.Page):
    n = 0

    t = ft.Text(color="white", size=150, weight=ft.FontWeight.W_200)
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
    page.add(t)

    while True:
        # Receive data
        data, addr = sock.recvfrom(1024)
        message = data.decode()
        print(message)
        t.value = message
        n += 1
        if n >= 500:
            n = 0
            page.update()

ft.app(target=main)
