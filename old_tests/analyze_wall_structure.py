#!/usr/bin/env python3
"""
Analisa a estrutura do grupo de paredes no arquivo OBJ
"""

import os

def analyze_wall_structure(input_file):
    """
    Analisa a estrutura interna do grupo de paredes
    """
    
    print(f"Analisando arquivo: {input_file}")
    
    with open(input_file, 'r', encoding='utf-8') as f:
        current_group = None
        current_object = None
        in_wall_group = False
        objects_in_wall = []
        face_count = 0
        
        for line in f:
            line = line.strip()
            
            if line.startswith('g '):
                current_group = line[2:]
                if 'parede' in current_group.lower():
                    in_wall_group = True
                    print(f"\nGrupo de parede encontrado: {current_group}")
                else:
                    in_wall_group = False
                    
            elif line.startswith('o ') and in_wall_group:
                current_object = line[2:]
                objects_in_wall.append(current_object)
                face_count = 0
                
            elif line.startswith('f ') and in_wall_group:
                face_count += 1
                
            elif line.startswith('g ') and in_wall_group:
                # Novo grupo, saindo do grupo de paredes
                if current_object and face_count > 0:
                    print(f"  Object: {current_object} - {face_count} faces")
                in_wall_group = False
        
        # Última verificação
        if current_object and face_count > 0 and in_wall_group:
            print(f"  Object: {current_object} - {face_count} faces")
    
    print(f"\nTotal de objetos no grupo de paredes: {len(objects_in_wall)}")
    print("\nLista de objetos (bodies) dentro do grupo de paredes:")
    for i, obj in enumerate(objects_in_wall, 1):
        print(f"{i}. {obj}")

def extract_wall_with_bodies(input_file, output_file):
    """
    Extrai o grupo de paredes mantendo a estrutura de objetos
    """
    print(f"\nExtraindo grupo de paredes completo...")
    
    vertices = []
    wall_content = []
    in_wall_group = False
    
    # Primeiro, ler todos os vértices
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(line)
    
    # Agora extrair o conteúdo do grupo de paredes
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('g '):
                if 'parede' in line.lower():
                    in_wall_group = True
                    wall_content.append(line)
                else:
                    in_wall_group = False
                    
            elif in_wall_group:
                # Captura tudo dentro do grupo de paredes
                if line.strip() and not line.startswith('v '):
                    wall_content.append(line)
    
    # Escrever arquivo de saída
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# Grupo de paredes extraído do escritório\n")
        f.write("# Mantendo estrutura de objetos (bodies)\n\n")
        
        # Vértices
        f.write("# Vértices\n")
        for v in vertices:
            f.write(v)
        
        f.write("\n# Grupo de paredes com objetos\n")
        for line in wall_content:
            f.write(line)
    
    print(f"Arquivo salvo: {output_file}")

def main():
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    
    if not os.path.exists(input_file):
        print(f"Erro: Arquivo '{input_file}' não encontrado!")
        return
    
    # Primeiro analisar a estrutura
    analyze_wall_structure(input_file)
    
    # Depois extrair mantendo a estrutura
    output_file = "3d escritorio/walls_with_bodies.obj"
    extract_wall_with_bodies(input_file, output_file)

if __name__ == "__main__":
    main()