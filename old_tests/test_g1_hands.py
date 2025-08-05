#!/usr/bin/env python3
"""
Script de teste para validar G1 com mãos completas de 5 dedos
"""

import subprocess
import time
import os
from pathlib import Path

def test_g1_simulation():
    """Testa simulação do G1 com mãos completas"""
    
    print("🤖 Testando simulação Unitree G1 com mãos completas (29 DOF)")
    print("="*60)
    
    # Diretórios
    sim_dir = Path("/home/pedro_setubal/Workspaces/G1/unitree_mujoco/simulate")
    build_dir = sim_dir / "build"
    config_file = sim_dir / "config.yaml"
    
    # Verificar se simulador está compilado
    if not (build_dir / "unitree_mujoco").exists():
        print("❌ Simulador não encontrado! Compile primeiro:")
        print(f"   cd {sim_dir}")
        print("   mkdir build && cd build")
        print("   cmake .. && make -j4")
        return False
    
    # Criar configuração temporária para G1 com mãos
    config_content = """robot: "g1"
robot_scene: "scene_29dof_with_hand.xml"
domain_id: 1
interface: "lo"
use_joystick: 0
joystick_type: "xbox"
joystick_device: "/dev/input/js0"
joystick_bits: 16
print_scene_information: 1
enable_elastic_band: 1
"""
    
    config_test = sim_dir / "config_g1_hands_test.yaml"
    with open(config_test, 'w') as f:
        f.write(config_content)
    
    print("✅ Configuração criada:")
    print(f"   Robot: G1")
    print(f"   Scene: scene_29dof_with_hand.xml (mãos completas)")
    print(f"   Elastic band: Habilitado")
    print()
    
    # Comandos para executar
    print("📋 Para executar a simulação:")
    print(f"   cd {build_dir}")
    print(f"   ./unitree_mujoco -c ../config_g1_hands_test.yaml")
    print()
    print("🎮 Controles na simulação:")
    print("   Mouse: Rotacionar câmera")
    print("   Shift+Mouse: Pan")
    print("   Scroll: Zoom")
    print("   Tecla '9': Ativar/desativar elástico")
    print("   Tecla '8': Levantar robô")
    print("   Tecla '7': Abaixar robô")
    print()
    
    # Criar script de execução
    run_script = sim_dir / "run_g1_hands.sh"
    with open(run_script, 'w') as f:
        f.write(f"""#!/bin/bash
cd {build_dir}
./unitree_mujoco -c ../config_g1_hands_test.yaml
""")
    os.chmod(run_script, 0o755)
    
    print("✅ Script de execução criado:")
    print(f"   {run_script}")
    print()
    
    return True

def check_g1_model():
    """Verifica se modelo G1 com mãos existe"""
    
    g1_dir = Path("/home/pedro_setubal/Workspaces/G1/unitree_mujoco/unitree_robots/g1")
    
    print("🔍 Verificando arquivos do G1:")
    
    # Arquivos essenciais
    required_files = [
        "g1_29dof_with_hand.xml",
        "scene_29dof_with_hand.xml",
        "meshes/left_hand_thumb_0_link.STL",
        "meshes/left_hand_thumb_1_link.STL", 
        "meshes/left_hand_thumb_2_link.STL",
        "meshes/left_hand_index_0_link.STL",
        "meshes/left_hand_index_1_link.STL",
        "meshes/left_hand_middle_0_link.STL",
        "meshes/left_hand_middle_1_link.STL",
    ]
    
    all_found = True
    for file in required_files:
        path = g1_dir / file
        if path.exists():
            print(f"   ✅ {file}")
        else:
            print(f"   ❌ {file} - NÃO ENCONTRADO!")
            all_found = False
    
    if all_found:
        print("\n✅ Todos os arquivos necessários encontrados!")
    else:
        print("\n❌ Alguns arquivos estão faltando!")
        
    return all_found

def create_test_controller():
    """Cria controlador de teste simples"""
    
    test_script = Path("/home/pedro_setubal/Workspaces/G1/test_g1_control.py")
    
    content = '''#!/usr/bin/env python3
"""
Controlador de teste para G1 com mãos - Envia comandos básicos
"""

import time
import numpy as np

# Tentar importar SDK (se disponível)
try:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
    SDK_AVAILABLE = True
except:
    print("SDK não disponível - modo simulação apenas")
    SDK_AVAILABLE = False

def test_hand_control():
    """Testa controle básico das mãos"""
    
    if not SDK_AVAILABLE:
        print("Execute este script após instalar unitree_sdk2_python")
        return
    
    # Inicializar para simulação
    ChannelFactoryInitialize(1, "lo")
    
    # Publisher para comandos low-level
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    
    cmd = unitree_hg_msg_dds__LowCmd_()
    
    print("Enviando comandos para as mãos...")
    
    # Índices das juntas das mãos (23-28)
    # 23-25: Mão esquerda
    # 26-28: Mão direita
    
    for i in range(100):
        # Movimento suave de abrir/fechar mãos
        angle = 0.3 * np.sin(i * 0.1)
        
        for joint_id in range(23, 29):  # Juntas das mãos
            cmd.motor_cmd[joint_id].mode = 0x01  # Modo servo
            cmd.motor_cmd[joint_id].q = angle
            cmd.motor_cmd[joint_id].kp = 10.0
            cmd.motor_cmd[joint_id].kd = 1.0
            cmd.motor_cmd[joint_id].tau = 0.0
        
        pub.Write(cmd)
        time.sleep(0.02)
    
    print("Teste concluído!")

if __name__ == "__main__":
    test_hand_control()
'''
    
    with open(test_script, 'w') as f:
        f.write(content)
    os.chmod(test_script, 0o755)
    
    print(f"✅ Controlador de teste criado: {test_script}")

if __name__ == "__main__":
    print("🚀 Configurando teste do G1 com mãos completas")
    print()
    
    # Verificar modelo
    if check_g1_model():
        print()
        # Configurar simulação
        if test_g1_simulation():
            print()
            # Criar controlador
            create_test_controller()
            print()
            print("✅ CONFIGURAÇÃO COMPLETA!")
            print()
            print("🎯 Próximos passos:")
            print("1. Execute a simulação:")
            print("   cd unitree_mujoco/simulate/build")
            print("   ./unitree_mujoco -c ../config_g1_hands_test.yaml")
            print()
            print("2. Verifique no log se aparece:")
            print("   - 29 joints (DOF)")
            print("   - Juntas das mãos listadas")
            print()
            print("3. Use o elástico virtual:")
            print("   - Tecla '9' para ativar")
            print("   - Tecla '8' para levantar o robô")
            print()
            print("4. Após validar, podemos prosseguir com as paredes!")