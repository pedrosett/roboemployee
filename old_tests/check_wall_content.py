#!/usr/bin/env python3
"""
Verifica o conteúdo do grupo de paredes
"""

def check_wall_content():
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    
    with open(input_file, 'r', encoding='utf-8') as f:
        in_wall_group = False
        line_count = 0
        vertex_count = 0
        face_count = 0
        
        for line in f:
            if line.startswith('g mesa parede tvs'):
                in_wall_group = True
                print("Encontrado grupo 'mesa parede tvs'")
                print("\nPrimeiras 30 linhas do grupo:")
                continue
                
            if in_wall_group:
                if line.startswith('g ') and not 'mesa parede tvs' in line:
                    # Novo grupo, parar
                    break
                    
                line_count += 1
                if line_count <= 30:
                    print(f"{line_count}: {line.strip()}")
                
                if line.startswith('v '):
                    vertex_count += 1
                elif line.startswith('f '):
                    face_count += 1
        
        print(f"\nResumo do grupo 'mesa parede tvs':")
        print(f"Total de linhas: {line_count}")
        print(f"Vértices (v): {vertex_count}")
        print(f"Faces (f): {face_count}")

if __name__ == "__main__":
    check_wall_content()