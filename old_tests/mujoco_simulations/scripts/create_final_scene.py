#!/usr/bin/env python3
import xml.etree.ElementTree as ET

# Ler os dois XMLs
office_content = """<mujoco model="office_only">
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="wall" rgba="0.9 0.9 0.9 1" specular="0.1"/>
    <mesh name="office_walls" file="3d escritorio/body1_structure/body1_structure.obj" scale="0.01 0.01 0.01"/>
  </asset>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" directional="true"/>
    <light pos="10 10 8" dir="-1 -1 -1" directional="false"/>
    <geom name="floor" size="50 50 0.05" type="plane" material="groundplane"/>
    <body name="walls" pos="-5.73 -7.00 0">
      <geom name="office_walls_geom" mesh="office_walls" type="mesh" contype="1" conaffinity="1" material="wall"/>
    </body>
  </worldbody>
</mujoco>"""

# Criar o XML final
with open('g1_5fingers_office_FINAL.xml', 'w') as f:
    f.write('<mujoco model="g1_5fingers_office_FINAL">\n')
    f.write('  <compiler angle="radian" autolimits="true"/>\n')
    f.write('  \n')
    f.write('  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic" jacobian="dense" solver="Newton" iterations="50"/>\n')
    f.write('  \n')
    f.write('  <statistic center="0 0 2" extent="20"/>\n')
    f.write('  \n')
    f.write('  <visual>\n')
    f.write('    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>\n')
    f.write('    <rgba haze="0.15 0.25 0.35 1"/>\n')
    f.write('    <global azimuth="-130" elevation="-20"/>\n')
    f.write('  </visual>\n')
    f.write('  \n')
    
    # Assets
    f.write('  <asset>\n')
    f.write('    <!-- Assets do ambiente -->\n')
    f.write('    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>\n')
    f.write('    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"\n')
    f.write('      markrgb="0.8 0.8 0.8" width="300" height="300"/>\n')
    f.write('    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>\n')
    f.write('    <material name="wall" rgba="0.9 0.9 0.9 1" specular="0.1"/>\n')
    f.write('    \n')
    f.write('    <!-- Paredes do escritório -->\n')
    f.write('    <mesh name="office_walls" file="3d escritorio/body1_structure/body1_structure.obj" scale="0.01 0.01 0.01"/>\n')
    f.write('  </asset>\n')
    f.write('  \n')
    
    # Worldbody
    f.write('  <worldbody>\n')
    f.write('    <!-- Iluminação -->\n')
    f.write('    <light pos="0 0 10" dir="0 0 -1" directional="true"/>\n')
    f.write('    <light pos="10 10 8" dir="-1 -1 -1" directional="false"/>\n')
    f.write('    \n')
    f.write('    <!-- Chão -->\n')
    f.write('    <geom name="floor" size="50 50 0.05" type="plane" material="groundplane"/>\n')
    f.write('    \n')
    f.write('    <!-- Paredes do escritório -->\n')
    f.write('    <body name="walls" pos="-5.73 -7.00 0">\n')
    f.write('      <geom name="office_walls_geom" mesh="office_walls" type="mesh" contype="1" conaffinity="1" material="wall"/>\n')
    f.write('    </body>\n')
    f.write('    \n')
    f.write('    <!-- G1 Robot com ajuste de altura -->\n')
    f.write('    <body name="robot_position" pos="0 0 0.80">\n')
    f.write('      <include file="g1_5fingers_converted.xml"/>\n')
    f.write('    </body>\n')
    f.write('  </worldbody>\n')
    f.write('</mujoco>\n')

print("✅ Criado: g1_5fingers_office_FINAL.xml")