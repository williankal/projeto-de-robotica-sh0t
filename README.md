Nome do grupo: 

____________

Nome dos integrantes: 

* Willian Kenzo Asanuma Lee
* Maria Eduarda Gonçalves Torres
* Renato Laffranchi Falcão
* Gabriel Hideki Stanzani Onishi



# Instruções

Comandos para atualizar os repositório
```bash
    cd ~catkin_ws/src/mybot_description
    git pull
    cd ~catkin_ws/src/my_simulation
    git pull
    cd ~catkin_ws/src/robot21.2
    git pull
```


Para executar:

	roslaunch my_simulation trevo.launch

Para habilitar o controle da garra executar:

	roslaunch mybot_description mybot_control2.launch 

Para rodar o script princiap:

	rosrun projeto-mwrg projeto.py
