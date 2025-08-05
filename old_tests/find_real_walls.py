#!/usr/bin/env python3
"""
Busca por paredes reais no arquivo OBJ original
"""

import re

def find_real_walls():
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    
    print("=== Buscando Paredes no Arquivo Original ===\n")
    
    # Padrões para identificar paredes
    wall_patterns = [
        r'wall', r'parede', r'pared', r'muro', r'panel',
        r'ceiling', r'teto', r'floor', r'piso', r'chao'
    ]
    
    groups = []
    current_group = None
    group_stats = {}
    
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('g '):
                current_group = line.strip()[2:]
                groups.append(current_group)
                group_stats[current_group] = {'vertices': 0, 'faces': 0}
            elif line.startswith('v ') and current_group:
                group_stats[current_group]['vertices'] += 1
            elif line.startswith('f ') and current_group:
                group_stats[current_group]['faces'] += 1
    
    print(f"Total de grupos encontrados: {len(groups)}\n")
    
    # Buscar grupos que parecem ser paredes
    print("=== Grupos com Palavras-chave de Parede ===")
    wall_candidates = []
    for group in groups:
        group_lower = group.lower()
        for pattern in wall_patterns:
            if pattern in group_lower:
                wall_candidates.append(group)
                print(f"- {group} (V: {group_stats[group]['vertices']}, F: {group_stats[group]['faces']})")
                break
    
    # Buscar grupos grandes que podem ser paredes
    print("\n=== Maiores Grupos (possíveis estruturas) ===")
    sorted_groups = sorted(group_stats.items(), key=lambda x: x[1]['faces'], reverse=True)
    
    for i, (group, stats) in enumerate(sorted_groups[:20]):
        print(f"{i+1}. {group}")
        print(f"   Vértices: {stats['vertices']}, Faces: {stats['faces']}")
    
    # Salvar lista completa de grupos
    with open("3d escritorio/all_groups.txt", "w", encoding='utf-8') as f:
        f.write("=== TODOS OS GRUPOS DO ARQUIVO OBJ ===\n\n")
        for i, (group, stats) in enumerate(sorted_groups):
            f.write(f"{i+1}. {group}\n")
            f.write(f"   Vértices: {stats['vertices']}, Faces: {stats['faces']}\n\n")
    
    print("\n✓ Lista completa salva em: 3d escritorio/all_groups.txt")
    
    return wall_candidates, sorted_groups

def extract_large_structures(num_groups=5):
    """
    Extrai os maiores grupos que provavelmente são estruturas/paredes
    """
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    output_file = "3d escritorio/large_structures.obj"
    
    # Primeiro, identificar os maiores grupos
    _, sorted_groups = find_real_walls()
    target_groups = [g[0] for g in sorted_groups[:num_groups]]
    
    print(f"\n=== Extraindo {num_groups} Maiores Estruturas ===")
    for g in target_groups:
        print(f"- {g}")
    
    vertices = []
    structure_content = []
    in_target_group = False
    current_group = None
    
    # Ler vértices
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(line)
    
    # Extrair conteúdo dos grupos alvo
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('g '):
                current_group = line.strip()[2:]
                if current_group in target_groups:
                    in_target_group = True
                    structure_content.append(line)
                else:
                    in_target_group = False
            elif in_target_group and not line.startswith('v '):
                structure_content.append(line)
    
    # Salvar arquivo
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# Maiores estruturas extraídas do escritório\n\n")
        f.write("# Vértices\n")
        for v in vertices:
            f.write(v)
        f.write("\n# Estruturas\n")
        for line in structure_content:
            f.write(line)
    
    print(f"\n✓ Estruturas salvas em: {output_file}")

if __name__ == "__main__":
    find_real_walls()
    extract_large_structures(10)  # Extrair os 10 maiores grupos