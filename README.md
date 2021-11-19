Nome do grupo: 

____________

Nome dos integrantes: 

* Willian Kenzo Asanuma Lee
* Maria Eduarda Gonçalves Torres
* Renato Laffranchi Falcão
* Gabriel Hideki Stanzani Onishi

## Conceito esperado: B

# Especiais realizados:
    - Uso de classes e objetos Python em todos os arquivos criados para o projeto
    - Fazer um controle proporcional derivativo ou PD para manter o robô na pista e fazer funcionar rápido baseado no ângulo de visão da pista.
    - Estruturar o programa com pelo menos um node ROS prestando serviço para o outro.

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

Para rodar o projeto:

	rosrun projeto-mwrg image.py  [cor] [id] [base]
	rosrun projeto-mwrg control.py
	
Visite o enunciado abaixo do projeto:

	https://github.com/Insper/robot21.2/blob/main/projeto/projeto.md

