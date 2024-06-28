from PIL import Image
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
# Cargar la nueva imagen
image_path = 'imagen.png'
image = Image.open(image_path).convert('RGB')  # Convertir a escala de grises
image_array = np.array(image)

# Convertir a matriz binaria (thresholding)
threshold = 128
binary_image = (image_array > threshold).astype(int)

# Mostrar la nueva imagen binaria
plt.imshow(binary_image, cmap='gray')
plt.title('Nueva Imagen de Obstáculos')
plt.show()

# Cargar el archivo .mat existente
mat_file_path = 'Environment1.mat'
mat_data = scipy.io.loadmat(mat_file_path)

# Actualizar la matriz BW
mat_data['BW'] = binary_image

# Guardar el archivo .mat actualizado
scipy.io.savemat('Environment1_new.mat', mat_data)
