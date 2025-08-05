#!/usr/bin/env python3
import xml.etree.ElementTree as ET

# Ler o XML do robô
robot_tree = ET.parse('g1_5fingers_converted.xml')
robot_root = robot_tree.getroot()

# Criar novo XML para a cena completa
scene_root = ET.Element('mujoco', model='g1_5fingers_office_scene')

# Compiler
compiler = ET.SubElement(scene_root, 'compiler', angle='radian', autolimits='true', 
                         meshdir='unitree_ros/robots/g1_description/meshes/')

# Option
option = ET.SubElement(scene_root, 'option', timestep='0.002', gravity='0 0 -9.81',
                       cone='elliptic', jacobian='dense', solver='Newton', iterations='50')

# Statistic
statistic = ET.SubElement(scene_root, 'statistic', center='0 0 2', extent='20')

# Visual
visual = ET.SubElement(scene_root, 'visual')
ET.SubElement(visual, 'headlight', diffuse='0.6 0.6 0.6', ambient='0.3 0.3 0.3', specular='0 0 0')
ET.SubElement(visual, 'rgba', haze='0.15 0.25 0.35 1')
ET.SubElement(visual, 'global', azimuth='-130', elevation='-20')

# Asset
asset = ET.SubElement(scene_root, 'asset')

# Adicionar assets do ambiente
ET.SubElement(asset, 'texture', type='skybox', builtin='gradient', 
              rgb1='0.3 0.5 0.7', rgb2='0 0 0', width='512', height='3072')
ET.SubElement(asset, 'texture', type='2d', name='groundplane', builtin='checker',
              mark='edge', rgb1='0.2 0.3 0.4', rgb2='0.1 0.2 0.3',
              markrgb='0.8 0.8 0.8', width='300', height='300')
ET.SubElement(asset, 'material', name='groundplane', texture='groundplane',
              texuniform='true', texrepeat='5 5', reflectance='0.2')
ET.SubElement(asset, 'material', name='wall', rgba='0.9 0.9 0.9 1', specular='0.1')

# Mesh das paredes
ET.SubElement(asset, 'mesh', name='office_walls', 
              file='../../../3d escritorio/body1_structure/body1_structure.obj',
              scale='0.01 0.01 0.01')

# Copiar todos os assets do robô
robot_assets = robot_root.find('asset')
if robot_assets is not None:
    for mesh in robot_assets.findall('mesh'):
        asset.append(mesh)

# Worldbody
worldbody = ET.SubElement(scene_root, 'worldbody')

# Iluminação
ET.SubElement(worldbody, 'light', pos='0 0 10', dir='0 0 -1', directional='true')
ET.SubElement(worldbody, 'light', pos='10 10 8', dir='-1 -1 -1', directional='false')

# Chão
ET.SubElement(worldbody, 'geom', name='office_floor', size='50 50 0.05', 
              type='plane', material='groundplane')

# Paredes
walls_body = ET.SubElement(worldbody, 'body', name='walls', pos='-5.73 -7.00 0')
ET.SubElement(walls_body, 'geom', name='office_walls_geom', mesh='office_walls',
              type='mesh', contype='1', conaffinity='1', material='wall')

# Copiar o body do robô com ajuste de posição
robot_worldbody = robot_root.find('worldbody')
if robot_worldbody is not None:
    for body in robot_worldbody:
        if body.tag == 'body':
            # Ajustar posição do pelvis
            body.set('pos', '0 0 0.80')
            worldbody.append(body)
            break

# Salvar o XML combinado
tree = ET.ElementTree(scene_root)
ET.indent(tree, space='  ')
tree.write('g1_5fingers_office_scene_combined.xml', encoding='utf-8', xml_declaration=True)

print("✅ XML combinado criado: g1_5fingers_office_scene_combined.xml")