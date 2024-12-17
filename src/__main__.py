import cv2
from vision.vision import *
from ml.model_predictor import *
from ros.controller import *
from ros.camera_subscriber import *
from ros.effort_trigger import *
import rospy
import time

if __name__ == "__main__":
    rospy.init_node("nodo_principal", anonymous=True)

    # Consumir imagen de la cámara
    nc = NodoCamara()

    # Suscripción al tópico de fuerza
    nf = NodoFuerza(umbral_fuerza=5.0)

    # Cargar modelos
    clf, le = load_models('src/ml/model/random_forest_model.pkl', 'src/ml/model/label_encoder.pkl')

    # Cooldown entre detecciones
    COOLDOWN_TIME = 3.0
    last_action_time = 0

    try:
        while not rospy.is_shutdown():
            ret, frame = nc.read_image()
            if not ret:
                print("No se pudo capturar el frame.")
                continue

            # Mostrar imagen en vivo
            cv2.imshow('Camara', frame)
            key = cv2.waitKey(1) & 0xFF

            # Detectar golpe o presionar tecla 'c'
            current_time = time.time()
            if (key == ord('c') or nf.golpe_detectado()) and (current_time - last_action_time > COOLDOWN_TIME):
                # Actualizar el tiempo de la última acción
                last_action_time = current_time

                rospy.loginfo("Iniciando procesamiento...")
                bright_image, blurred, edges, contours_image, contours = process_image(frame)

                if contours:
                    # Extraer características
                    rugosity = calculate_rugosity(bright_image)
                    shape_features = calculate_shape_features(contours)
                    contrast, correlation, energy, homogeneity = calculate_glcm_features(bright_image)
                    color_ranges = get_color_ranges(bright_image, contours)

                    if shape_features and color_ranges:
                        # Crear diccionario de características
                        features = {**shape_features, 'rugosity': rugosity, 'contrast': contrast, 
                                    'correlation': correlation, 'energy': energy, 'homogeneity': homogeneity}
                        
                        for i, h_range in enumerate(color_ranges[:5]):
                            features[f'hue_range_{i+1}_start'] = h_range[0]
                            features[f'hue_range_{i+1}_end'] = h_range[1]

                        # Predecir clase
                        prediction = predict_class(clf, le, features)
                        print(f"Predicción: {prediction}")

                        # Mover fruta
                        if "defecto" in prediction:
                            poner_caja_mala()
                        else:
                            poner_caja_buena()
                else:
                    print("No se detectaron contornos.")
                
                # Esperar un tiempo después de la acción
                rospy.sleep(COOLDOWN_TIME)

            # Salir al presionar 'q'
            elif key == ord('q'):  
                break

    except Exception as e:
        print(f"Se ha producido un error durante la ejecución: {e}")

    finally:
        cv2.destroyAllWindows()
        print("Cámara liberada y ventanas cerradas.")
