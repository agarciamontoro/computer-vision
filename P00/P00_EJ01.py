import cv2
import numpy as np

def leeimagen(file_name, flag_color=True):
    #Valor por defecto
    cv_flag = cv2.IMREAD_COLOR

    if flag_color == False:
        cv_flag =cv2.IMREAD_GRAYSCALE

    return cv2.imread(file_name, cv_flag)

def pintaI(img):
    cv2.namedWindow("Imagen")
    cv2.imshow("Imagen", img)

def pintaMI(vim):
    width = 0
    height = 0

    #Calculamos el ancho de la imagen final sumando
    #los anchos de todas las imagenes. Ademas, calculamos
    #el alto de la imagen final como el alto maximo de
    #todas las imagenes
    for img in vim:
        width += img.shape[0]
        if img.shape[1] > height: height = img.shape[1]

    big_img = np.zeros((height, width, 3), np.uint8)

    width = 0

    for img in vim:
        #Si es ByN, trasladamos esa imagen a tres canales copiando el mismo valor
        #en cada uno de ellos
        if len(img.shape) < 3:
            img = cv2.merge([img,img,img])

        #Copiamos la imagen actual en el ROI de la imagen destino
        big_img[:, width:width+img.shape[0],:] = img

        #Actualizamos el valor del ancho para el proximo ROI
        width += img.shape[0]

    pintaI(big_img)

def setPXvalue(img, px_coord, value=[0,0,255]):
    for px in px_coord:
        img[px[0],px[1],:] = value


####################################################################

if __name__ == "__main__":
    #Leemos la misma imagen en modo gris y modo color
    imagen_gris  = leeimagen("../IMG/lena.jpg", False)
    imagen_color = leeimagen("../IMG/lena.jpg")

    #Creamos una lista de coordenadas para usar en la modificacion de la imagen:
    #basicamente, es una malla de puntos con 5 px de distancia entre ellos
    coordinates = [[x,y] for x in range(0,256,5) for y in range(0,256,5)]

    #Copiamos la imagen de color en una nueva y la modificamos
    imagen_color_mod = np.copy(imagen_color)
    setPXvalue(imagen_color_mod, coordinates, [255,255,255])

    #Visualizamos las tres imagenes en una sola ventana
    pintaMI([imagen_color, imagen_gris, imagen_color_mod])

    #Esperamos a que el usuario pulse 'ESC'
    while True:
        k = cv2.waitKey(0) & 0xFF    # 0xFF? To get the lowest byte.
        if k == 27: break            # Code for the ESC key

    cv2.destroyAllWindows()
