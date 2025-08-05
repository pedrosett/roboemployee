#!/usr/bin/env python3
import xml.etree.ElementTree as ET

# Ler o XML do robô de 5 dedos
print("Lendo XML do robô de 5 dedos...")
robot_tree = ET.parse('g1_5fingers_converted.xml')
robot_root = robot_tree.getroot()

# Ler o XML do escritório com robô de 3 dedos
print("Lendo XML do escritório...")
office_tree = ET.parse('g1_office_unified.xml')
office_root = office_tree.getroot()

# Atualizar o model name
office_root.set('model', 'g1_5fingers_office_unified')

# Atualizar o meshdir no compiler para os meshes do robô
compiler = office_root.find('compiler')
if compiler is not None:
    compiler.set('meshdir', 'unitree_ros/robots/g1_description/meshes/')

# Substituir os meshes do robô
print("Atualizando assets...")
office_assets = office_root.find('asset')
robot_assets = robot_root.find('asset')

# Remover meshes antigos do robô (mantendo texturas e paredes)
meshes_to_remove = []
for mesh in office_assets.findall('mesh'):
    name = mesh.get('name')
    if name and name != 'office_walls':
        meshes_to_remove.append(mesh)

for mesh in meshes_to_remove:
    office_assets.remove(mesh)

# Adicionar meshes do robô de 5 dedos
for mesh in robot_assets.findall('mesh'):
    new_mesh = ET.Element('mesh')
    for attr, value in mesh.attrib.items():
        new_mesh.set(attr, value)
    office_assets.append(new_mesh)

# Ajustar path das paredes para ser relativo ao novo meshdir
walls_mesh = None
for mesh in office_assets.findall('mesh'):
    if mesh.get('name') == 'office_walls':
        walls_mesh = mesh
        break
        
if walls_mesh is not None:
    walls_mesh.set('file', '../../../3d escritorio/body1_structure/body1_structure.obj')

# Agora substituir o corpo do robô no worldbody
print("Atualizando corpo do robô...")
office_worldbody = office_root.find('worldbody')
robot_worldbody = robot_root.find('worldbody')

# Encontrar e remover o body do pelvis antigo
for body in list(office_worldbody):
    if body.tag == 'body' and body.get('name') == 'pelvis':
        office_worldbody.remove(body)
        break

# Adicionar o novo body do robô de 5 dedos
for body in robot_worldbody:
    if body.tag == 'body':
        # Ajustar posição para 0.80m
        body.set('pos', '0 0 0.80')
        office_worldbody.append(body)
        break

# Salvar o arquivo final
print("Salvando arquivo final...")
ET.indent(office_tree, space='  ')
office_tree.write('g1_5fingers_office_unified_FINAL.xml', encoding='utf-8', xml_declaration=True)

print("✅ Criado: g1_5fingers_office_unified_FINAL.xml")
print("   - Robô G1 com mãos de 5 dedos")
print("   - Ambiente do escritório com paredes")
print("   - Posição ajustada para 0.80m")