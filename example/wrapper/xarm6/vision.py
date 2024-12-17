import cv2
from pyzbar.pyzbar import decode

def main():
    cap = cv2.VideoCapture(0)  # Iniciar la cámara web (cambia el índice si tienes varias cámaras)

    while True:
        ret, frame = cap.read()  # Capturar un fotograma de la cámara
        if not ret:
            break

        decoded_objects = decode(frame)  # Decodificar objetos QR en el fotograma

        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')  # Decodificar los datos del QR
            qr_type = obj.type

            print(f'Tipo de QR: {qr_type}')
            print(f'Datos del QR: {qr_data}')

        cv2.imshow('QR Reader', frame)  # Mostrar el fotograma con el cuadro del QR resaltado

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()  # Liberar la cámara
    cv2.destroyAllWindows()  # Cerrar todas las ventanas

if __name__ == "__main__":
    main()