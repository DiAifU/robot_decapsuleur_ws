# Robot décapsuleur
##### CPE Lyon : Projet libre de prototypage transversal sur 20h.

## Le robot :
Notre prototype est un robot décapsuleur permettant de décapsuler une bouteille de bière qu'une personne tend au robot. On considère qu'une bière est à portée du robot à une distance inférieure à 30 cm. Le robot évoluera dans un environnement dégagé.

## Projet GitHub : [lien](https://github.com/DiAifU/robot_decapsuleur_ws.git)

## Matériel utilisé :
- Un lidar
- 4 moteurs dynamixels
- Une carte Arduino
- Une caméra logitech
- 1 module BLE
- Un bras et un décapsuleur imprimés en 3D : [liens](https://github.com/DiAifU/robot_decapsuleur_ws/tree/master/src/robot_decapsuleur_description/stl)

## Principe de fonctionnement :
- Le fonctionnement de notre robot repose sur la détection d'objets effectuée par le LIDAR. Ce dernier va détecter l'objet le plus proche.
- Le robot place ensuite la caméra face à l'objet en question.
- Grâce à de l'analyse d'image, le robot vérifie que l'objet est une bière.
- A l'aide du Lidar, le robot récupère les coordonnées de la capsule.
- Le bras du robot place ensuite le décapsuleur (effecteur) sur la capsule avant de la décapsuler.

Par ailleurs, une application mobile permet de mettre en marche le robot et de l'arrêter. Pour cela, un module Bluetooth Low Energy (BLE) relié à une carte de développement Arduino a été implémenté. Les données du module BLE sont ensuite transférés au roscore au moyen du liaison serial (package rosserial\_arduino).

## Mise en route :
- Créer un workspace
- Cloner ensuite les paquets du github.

Il faut que soient branchés :
- Le driver des 4 moteurs dynamixels
- Le lidar
- La caméra
- L'arduino avec module BLE

## Démarrage du robot : ```roslaunch robot_decapsuleur robot.launch```

Se connecter en Bluetooth (via le module BLE) et envoyer l'entier '1' pour marche et '0' pour mettre en mode Sleep le robot (application iPhone : LightBlue)

## Vidéo du robot : [lien](https://drive.google.com/drive/folders/1X8mjzTXQk2E1sC0soa0biHBy6vVwvcat?fbclid=IwAR3YxGDGco1k03rlCZHW8tWZq81bPCm7GggKrlCzyPDzNgAgbf-9rf8cUgM)

## Etudiants :
- Nicolas MONTVERNAY
- Jean-Baptiste RAMBAUD
- Thomas SEVEDE

## Professeurs encadrants :
- Raphael LEBER
- Fabrice JUMEL

CPE Lyon : [https://www.cpe.fr/](https://www.cpe.fr/)
