#!/usr/bin/env python3
"""
Extrai apenas as paredes do arquivo OBJ do escritório
"""

import os
import re

def extract_walls_from_obj(input_file, output_file):
    """
    Extrai grupos que parecem ser paredes baseado no nome
    """
    
    print(f"Lendo arquivo: {input_file}")
    
    wall_keywords = ['wall', 'parede', 'pared', 'muro', 'panel']
    current_group = None
    wall_groups = []
    vertices = []
    faces_to_write = []
    
    # Primeiro, ler todos os vértices
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(line)
    
    # Agora identificar grupos de paredes e suas faces
    with open(input_file, 'r', encoding='utf-8') as f:
        is_wall_group = False
        
        for line in f:
            if line.startswith('g '):
                # Novo grupo
                current_group = line.strip()
                group_name = current_group.lower()
                
                # Verificar se é uma parede
                is_wall_group = any(keyword in group_name for keyword in wall_keywords)
                
                if is_wall_group:
                    wall_groups.append(current_group)
                    print(f"Grupo de parede encontrado: {current_group}")
                    
            elif line.startswith('f ') and is_wall_group:
                # Face pertence a um grupo de parede
                faces_to_write.append(line)
    
    # Escrever arquivo de saída
    print(f"\nEscrevendo arquivo: {output_file}")
    print(f"Total de vértices: {len(vertices)}")
    print(f"Total de grupos de parede: {len(wall_groups)}")
    print(f"Total de faces de parede: {len(faces_to_write)}")
    
    with open(output_file, 'w', encoding='utf-8') as f:
        # Cabeçalho
        f.write("# Paredes extraídas do escritório\n")
        f.write(f"# Grupos de parede: {len(wall_groups)}\n\n")
        
        # Escrever todos os vértices (necessário para as faces)
        f.write("# Vértices\n")
        for v in vertices:
            f.write(v)
        
        f.write("\n# Faces das paredes\n")
        
        # Re-processar para manter organização por grupo
        with open(input_file, 'r', encoding='utf-8') as fin:
            is_wall_group = False
            
            for line in fin:
                if line.startswith('g '):
                    current_group = line.strip()
                    group_name = current_group.lower()
                    is_wall_group = any(keyword in group_name for keyword in wall_keywords)
                    
                    if is_wall_group:
                        f.write(f"\n{line}")
                        
                elif line.startswith('f ') and is_wall_group:
                    f.write(line)
    
    return wall_groups

def main():
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    output_file = "3d escritorio/walls_only.obj"
    
    if not os.path.exists(input_file):
        print(f"Erro: Arquivo '{input_file}' não encontrado!")
        return
    
    wall_groups = extract_walls_from_obj(input_file, output_file)
    
    if not wall_groups:
        print("\nATENÇÃO: Nenhum grupo de parede foi encontrado!")
        print("Vamos listar todos os grupos para análise manual...")
        
        # Listar todos os grupos
        with open(input_file, 'r', encoding='utf-8') as f:
            groups = []
            for line in f:
                if line.startswith('g '):
                    groups.append(line.strip())
            
        print(f"\nTotal de grupos no arquivo: {len(groups)}")
        print("\nPrimeiros 20 grupos:")
        for i, g in enumerate(groups[:20]):
            print(f"{i+1}. {g}")

if __name__ == "__main__":
    main()