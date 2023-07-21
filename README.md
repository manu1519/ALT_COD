# ALT_COD
Codigo que permite la comunicación serial entre controladora de vuelo con Firmaware Pixhawk y una CSG (Estación de control terrestre) por medio de la librería Dronekit. Cabe resaltar el uso de la misma tiene que estar basada en los comandos establecidos por MAVLink proveniente de la librería Ardupilot de tipo OpenSource. A la par se implementa un código que permite la comunicación con 2 servomotores para retraer y extender el trén de aterrizaje enfocado en la función principal del modelo QR X800 de la marca Walkera, no obstante, de esta forma, cualquier diseño con tren de aterrizaje, puede ser implementado, llevando los motores a 45°, todo esto por medio del sistema embebido Raspberry Pi 4. Estas acciones son implementadas por medio de la lectura de la altura que provee la controladora de vuelo Pixhawk.
