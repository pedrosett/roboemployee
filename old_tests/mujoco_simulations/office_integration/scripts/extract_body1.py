#!/usr/bin/env python3
"""
Extrai apenas o Body1 que parece ser uma estrutura grande (piso ou teto)
"""

def extract_body1():
    input_file = "3d escritorio/escritorio_CW_scan.obj"
    output_file = "3d escritorio/body1_structure.obj"
    
    print("=== Extraindo Body1 ===")
    
    vertices = []
    body1_content = []
    in_body1 = False
    
    # Ler todos os vértices
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(line)
    
    # Extrair apenas Body1
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('g '):
                current_group = line.strip()[2:]
                if current_group == 'Body1':
                    in_body1 = True
                    body1_content.append(line)
                else:
                    in_body1 = False
            elif in_body1 and not line.startswith('v '):
                body1_content.append(line)
    
    # Salvar
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# Body1 - Estrutura grande (possível piso/teto)\n\n")
        f.write("# Vértices\n")
        for v in vertices:
            f.write(v)
        f.write("\n# Body1\n")
        for line in body1_content:
            f.write(line)
    
    print(f"✓ Salvo em: {output_file}")
    print("Dimensões: 3.28m x 2.84m x 0.32m")
    print("Altura Z: 0-315mm")

if __name__ == "__main__":
    extract_body1()