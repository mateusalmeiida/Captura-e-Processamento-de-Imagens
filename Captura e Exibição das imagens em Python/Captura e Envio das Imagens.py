import cv2
import numpy as np
import serial

# Inicializa a comunicação serial
ser = serial.Serial('COM5', timeout=1)

# Inicializa a captura de vídeo da câmera
cap = cv2.VideoCapture(0)

# Verifica se a câmera foi aberta com sucesso
if not cap.isOpened():
    print("Erro ao abrir a câmera.")
    exit()

while True:
    # Captura um frame
    ret, frame = cap.read()

    # Verifica se o frame foi capturado com sucesso
    if ret:
    
        #Converte o frame para YUV
        frame_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)

        # Extrai o canal de luminância
        y_channel = frame_yuv[:, :, 0]

        # Divide o y_channel em blocos de 64 bytes
        block_size = 64
        y_channel_flat = y_channel.flatten()
        blocks = np.array_split(y_channel_flat, (len(y_channel_flat) + block_size - 1) // block_size)

        luminancia = []

        # Envia cada bloco via serial
        for idx, block in enumerate(blocks):
            ser.write(block.tobytes())  # Envia o bloco como bytes

            # Recebe os blocos modificados pelo STM32
            data = ser.read(64)  # Leitura de 64 bytes
            luminancia.append(data)

        luminancia = b''.join(luminancia) #concatena os blocos de bytes

        luminancia_flat = np.frombuffer(luminancia, dtype=np.uint8)  # Converte para array 1D de uint8
        luminancia_matrix = luminancia_flat.reshape(y_channel.shape)  # Remodela para a forma original

    
        frame_yuv[:, :, 0] = luminancia_matrix
        frame_yuv = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR)

        #Exibe a imagem original e a retocada
        cv2.imshow("Imagem modificada", frame_yuv)
        cv2.imshow("Imagem original", frame)
        # Verifica se a tecla 'q' foi pressionada para sair do loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    else:
        print("Erro ao capturar o frame.")


# Libera a captura e fecha as janelas
cap.release()
cv2.destroyAllWindows()
ser.close()  # Fecha a comunicação serial