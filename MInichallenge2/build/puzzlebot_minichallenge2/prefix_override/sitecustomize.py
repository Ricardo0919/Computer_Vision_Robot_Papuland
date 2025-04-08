import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ricardosierra/Documents/TEC/6Semestre/ImplementacionDeRoboticaInteligente/ManchesterRobotics/Computer_Vision_Robot_Papuland/MInichallenge2/install/puzzlebot_minichallenge2'
