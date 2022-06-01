[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7421864&assignment_repo_type=AssignmentRepo)
# RoboCup

Para esta práctica hemos realizado dos de las pruebas de la Robocup Junior, descritas a continuación:

## Carry my luggage

Esta prueba consistía en, desde un punto de inicio, conseguir llevar al Kobuki hasta la arena donde se encontraba el árbitro. Allí debía ubicarle correctamente, centrarlo, y detectar la maleta que este le indicase. Posteriormente debería seguirlo fuera de la arena, esquivando posibles obstáculos de distinto tamaño y sin perderlo ni desorientarse. Tras llegar a un punto fuera de la zona mapeada el árbitro debería indicar al robot que había llegado al destino y el Kobuki tendría que volver a la arena.

### Resultado

Para esta prueba hemos empleado principalmente:
1. Navegación, mediante una clase nav.cpp
2. Dialogo, por medio de una clase chatbot.cpp
3. Detecctión de personas por bounding boxes, en el nodo Follow_person del behavior tree

Dicho BT empleado ha sido el siguiente:
![Image text](https://github.com/Docencia-fmrico/robocup-home-education-nocom-pila/blob/main/raw/cml.png)

Finalmente conseguimos hacer que el robot comenzara su funcionamiento por voz, sin necesidad de un botón de inicio. Además, llegó correctamente a la arena, situándose también de forma correcta enfrente del árbitro. A pesar de algunos problemas con las interacciones por diálogo, se orientó hacia la maleta correcta que indicó por voz el árbitro. Tras esto, siguió al árbitro correctamente fuera de la arena y logró salir también de la zona mapeada, aunque con algunos inconvenientes (el PID daba ciertos tirones tal como lo empleamos y tenía leves complicaciones evitando obstáculos). La parte final de volver a la arena no fuimos capaces de terminarla a tiempo.


## Find my mates

Esta prueba consistía en, también desde un punto de partida, hacer que el Kobuki fuera a la arena y, en las posibles posiciones que había diferenciadas, buscar posibles personas. Una vez encontrada una persona debía ser reportada al árbitro, que se encontraba fuera de la arena, indicándole a este el nombre, color de la camiseta y objeto que portaba la misma. El máximo de personas a reportar eran tres, siendo la última una bonificación, y se podía ir una por una o hacer varias de una misma vez.

### Resultado

Para esta prueba hemos utilizado:
1. Navegación, empleando también la clase nav.cpp mencionada previamente
2. Diálogo, asimismo utilizado también en la prueba anterior mediante la calse chatbot.cpp
3. Detección de personas y objetos por medio de bounding boxes, en los nodos Detect_person y Detect_object de nuestro behavior tree

Dicho BT implementado lo podemos ver en la siguiente imagen:
![Image text](https://github.com/Docencia-fmrico/robocup-home-education-nocom-pila/blob/main/raw/fmm.png)

Durante la realización de la prueba, logramos hacer que el robot iniciase, de igual modo que antes, sin necesidad de u n botón de comienzo. Tras esto, se desplazó a la arena sufriendo en su primer trayecto algunos problemas con la navegación, pero llegando finalmente a la segunda posición de las posibles en que podía haber ubicadas personas. Detectó de forma correcta el nombre y color de la camiseta de la primera persona por diálogo, además de su localización. Del mismo modo, logramos reportar al árbitro una segunda persona, aunque para la tercera hubo problemas. Dado que la navegación abortó a mitad de camino, el robot no reportó a la persona frente al árbitro pero consiguió emitir la información correcta.
