#!/usr/bin/env python3
"""
Analisa os grupos Body para identificar possíveis paredes/estruturas
"""

import numpy as np

def analyze_body_groups():
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    
    # Grupos Body principais para analisar
    body_groups = ['Body1', 'Body2', 'Body3', 'Body4', 'Body7']
    
    print("=== Análise dos Grupos Body ===\n")
    
    for target_group in body_groups:
        vertices = []
        in_group = False
        
        with open(input_file, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('g '):
                    current_group = line.strip()[2:]
                    in_group = (current_group == target_group)
                elif in_group and line.startswith('v '):
                    parts = line.split()
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif line.startswith('g ') and in_group:
                    break
        
        if vertices:
            vertices = np.array(vertices)
            bounds = np.max(vertices, axis=0) - np.min(vertices, axis=0)
            
            print(f"\n{target_group}:")
            print(f"  Vértices: {len(vertices)}")
            print(f"  Dimensões (mm): X={bounds[0]:.1f}, Y={bounds[1]:.1f}, Z={bounds[2]:.1f}")
            print(f"  Dimensões (m): X={bounds[0]/1000:.2f}, Y={bounds[1]/1000:.2f}, Z={bounds[2]/1000:.2f}")
            
            # Análise de proporções para identificar paredes
            if bounds[2] > 1500:  # Mais de 1.5m de altura
                if bounds[0] < 300 or bounds[1] < 300:  # Fino em uma direção
                    print(f"  → POSSÍVEL PAREDE (altura: {bounds[2]/1000:.2f}m)")
            
            # Centro e posição
            center = np.mean(vertices, axis=0)
            print(f"  Centro: {center}")
            print(f"  Min Z: {np.min(vertices[:, 2]):.1f}mm")
            print(f"  Max Z: {np.max(vertices[:, 2]):.1f}mm")

def extract_body_groups():
    """
    Extrai os grupos Body principais
    """
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    output_file = "3d escritorio/body_structures.obj"
    
    target_groups = ['Body1', 'Body2', 'Body3', 'Body4', 'Body7']
    
    print("\n=== Extraindo Grupos Body ===")
    
    vertices = []
    body_content = []
    in_target = False
    current_group = None
    
    # Ler todos os vértices
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(line)
    
    # Extrair conteúdo dos Body groups
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('g '):
                current_group = line.strip()[2:]
                # Apenas os grupos principais, não os :1, :2, etc
                if current_group in target_groups:
                    in_target = True
                    body_content.append(line)
                else:
                    in_target = False
            elif in_target and not line.startswith('v '):
                body_content.append(line)
    
    # Salvar
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# Grupos Body extraídos\n\n")
        f.write("# Vértices\n")
        for v in vertices:
            f.write(v)
        f.write("\n# Estruturas Body\n")
        for line in body_content:
            f.write(line)
    
    print(f"✓ Salvo em: {output_file}")

if __name__ == "__main__":
    analyze_body_groups()
    extract_body_groups()