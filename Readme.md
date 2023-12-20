Controlador de Campo Potencial

Este pacote ROS contém código para controlar um robô para evitar um obstaculo usando controladores de Campos Potenciais.
Visão geral

O código consiste em dois nós:

     stageros: inicia o simulador de palco com um arquivo mundial especificado.
     move_example_node: Controla o robô para evitar obstaculos com campos potenciais.

Iniciar arquivo

O arquivo de inicialização configura parâmetros e inicia nós:

"a": Metade da largura da lemniscata
"kp": Parâmetro de ganho proporcional para o controlador PID
"kp1": Parâmetro de ganho proporcional 1 para o controlador PID
"kp2": Parâmetro de ganho proporcional 2 para o controlador PID
"d": Parâmetro de largura da lemniscata
"x_goal": Parametro de objetivo de posição X do robo
"y_goal": Parametro de objetivo de posição Y do robo
"d": Parametro de controle de Feedback Linearization

Observação

Certifique-se de que os pacotes necessários estejam instalados corretamente e que o arquivo de inicialização aponte para caminhos e parâmetros de arquivo válidos.
Notas

     Certifique-se de que os caminhos corretos sejam fornecidos no arquivo de inicialização para o simulador de palco e geração de trajetória.
     Modifique os parâmetros de ajuste do de Linearização de Feedback ou características de trajetória conforme necessário.

Use apt-get to install the Eigen3 library:
sudo apt update
sudo apt-get install libeigen3-dev

