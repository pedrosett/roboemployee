#!/usr/bin/env python3
"""
Cria uma cena unificada combinando o G1 e as paredes do escritório
"""

import os
import xml.etree.ElementTree as ET

def merge_xml_files():
    # Ler o arquivo do G1
    g1_file = "unitree_mujoco/unitree_robots/g1/g1_29dof_with_hand.xml"
    scene_file = "unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml"
    output_file = "g1_office_unified.xml"
    
    print("Criando cena unificada...")
    
    # Ler o conteúdo do G1
    with open(g1_file, 'r') as f:
        g1_content = f.read()
    
    # Criar o arquivo unificado
    unified_xml = f'''<mujoco model="g1_office_unified">
  <compiler angle="radian" autolimits="true"/>
  
  <statistic center="0 0 2" extent="20"/>
  
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>
  
  <!-- Classes do G1 -->
  <default>
    <default class="torso_motor">
      <joint damping="0.05" armature="0.01" frictionloss="0.2"/>
    </default>
    <default class="leg_motor">
      <joint damping="0.05" armature="0.01" frictionloss="0.2"/>
    </default>
    <default class="ankle_motor">
      <joint damping="0.05" armature="0.01" frictionloss="0.2"/>
    </default>
    <default class="arm_motor">
      <joint damping="0.05" armature="0.01" frictionloss="0.2"/>
    </default>
    <default class="wrist_motor">
      <joint damping="0.05" armature="0.01" frictionloss="0.1"/>
    </default>
    <default class="hand_motor">
      <joint damping="0.01" armature="0.01" frictionloss="0.1"/>
    </default>
  </default>
  
  <asset>
    <!-- Assets do ambiente -->
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="wall" rgba="0.9 0.9 0.9 1" specular="0.1"/>
    
    <!-- Paredes do escritório -->
    <mesh name="office_walls" file="3d escritorio/body1_structure/body1_structure.obj" scale="0.01 0.01 0.01"/>
    
    <!-- Meshes do G1 (todos os 74 meshes) -->
'''
    
    # Extrair os meshes do arquivo G1
    tree = ET.parse(g1_file)
    root = tree.getroot()
    
    # Encontrar todos os meshes
    asset_elem = root.find('asset')
    if asset_elem is not None:
        for mesh in asset_elem.findall('mesh'):
            name = mesh.get('name')
            file = mesh.get('file')
            unified_xml += f'    <mesh name="{name}" file="unitree_mujoco/unitree_robots/g1/meshes/{file}" />\n'
    
    unified_xml += '''  </asset>
  
  <worldbody>
    <!-- Iluminação -->
    <light pos="0 0 10" dir="0 0 -1" directional="true"/>
    <light pos="10 10 8" dir="-1 -1 -1" directional="false"/>
    
    <!-- Chão -->
    <geom name="floor" size="50 50 0.05" type="plane" material="groundplane"/>
    
    <!-- Paredes do escritório -->
    <body name="walls" pos="-5.73 -7.00 0">
      <geom name="office_walls_geom" mesh="office_walls" type="mesh" contype="1" conaffinity="1" material="wall"/>
    </body>
    
    <!-- G1 Robot -->
'''
    
    # Copiar a estrutura do worldbody do G1
    worldbody = root.find('worldbody')
    if worldbody is not None:
        # Converter para string e adicionar
        for body in worldbody:
            body_str = ET.tostring(body, encoding='unicode')
            # Ajustar indentação
            body_str = '    ' + body_str.replace('\n', '\n    ')
            unified_xml += body_str + '\n'
    
    unified_xml += '''  </worldbody>
  
  <!-- Sensores do G1 -->
'''
    
    # Copiar sensores se existirem
    sensor = root.find('sensor')
    if sensor is not None:
        unified_xml += '  <sensor>\n'
        for s in sensor:
            s_str = ET.tostring(s, encoding='unicode')
            unified_xml += '    ' + s_str + '\n'
        unified_xml += '  </sensor>\n'
    
    # Copiar atuadores
    actuator = root.find('actuator')
    if actuator is not None:
        unified_xml += '\n  <actuator>\n'
        for a in actuator:
            a_str = ET.tostring(a, encoding='unicode')
            unified_xml += '    ' + a_str + '\n'
        unified_xml += '  </actuator>\n'
    
    unified_xml += '</mujoco>'
    
    # Salvar arquivo
    with open(output_file, 'w') as f:
        f.write(unified_xml)
    
    print(f"Arquivo criado: {output_file}")
    print("\nPara executar:")
    print(f"  simulate {output_file}")

if __name__ == "__main__":
    merge_xml_files()