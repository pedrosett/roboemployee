#!/usr/bin/env python3
"""
Script simples para analisar estrutura básica do arquivo OBJ sem dependências externas
"""

import re
from pathlib import Path
from collections import defaultdict

def analyze_obj_structure(obj_path):
    """Analisa estrutura básica de um arquivo OBJ"""
    print(f"Analisando arquivo: {obj_path}")
    print("-" * 80)
    
    stats = defaultdict(int)
    groups = []
    objects = []
    materials_used = []
    mtl_libs = []
    
    # Padrões regex
    patterns = {
        'vertex': re.compile(r'^v\s+'),
        'vertex_texture': re.compile(r'^vt\s+'),
        'vertex_normal': re.compile(r'^vn\s+'),
        'face': re.compile(r'^f\s+'),
        'group': re.compile(r'^g\s+(.+)'),
        'object': re.compile(r'^o\s+(.+)'),
        'mtllib': re.compile(r'^mtllib\s+(.+)'),
        'usemtl': re.compile(r'^usemtl\s+(.+)'),
        'smooth': re.compile(r'^s\s+(.+)'),
    }
    
    group_face_count = defaultdict(int)
    current_group = None
    
    try:
        with open(obj_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line_num, line in enumerate(f):
                line = line.strip()
                
                # Contar elementos
                if patterns['vertex'].match(line):
                    stats['vertices'] += 1
                elif patterns['vertex_texture'].match(line):
                    stats['texture_coords'] += 1
                elif patterns['vertex_normal'].match(line):
                    stats['normals'] += 1
                elif patterns['face'].match(line):
                    stats['faces'] += 1
                    if current_group:
                        group_face_count[current_group] += 1
                
                # Grupos e objetos
                elif match := patterns['group'].match(line):
                    group_name = match.group(1).strip()
                    groups.append(group_name)
                    current_group = group_name
                    stats['groups'] += 1
                    
                elif match := patterns['object'].match(line):
                    object_name = match.group(1).strip()
                    objects.append(object_name)
                    stats['objects'] += 1
                
                # Materiais
                elif match := patterns['mtllib'].match(line):
                    mtl_file = match.group(1).strip()
                    mtl_libs.append(mtl_file)
                    
                elif match := patterns['usemtl'].match(line):
                    material = match.group(1).strip()
                    if material not in materials_used:
                        materials_used.append(material)
                
                # Mostrar primeiras ocorrências
                if line_num < 100 and any([
                    patterns['group'].match(line),
                    patterns['object'].match(line),
                    patterns['mtllib'].match(line),
                    patterns['usemtl'].match(line)
                ]):
                    print(f"  Linha {line_num:4d}: {line[:80]}")
                    
    except Exception as e:
        print(f"Erro ao ler arquivo: {e}")
        return None
    
    # Resultados
    print(f"\n📊 ESTATÍSTICAS:")
    print(f"  Vértices (v): {stats['vertices']:,}")
    print(f"  Coordenadas de textura (vt): {stats['texture_coords']:,}")
    print(f"  Normais (vn): {stats['normals']:,}")
    print(f"  Faces (f): {stats['faces']:,}")
    print(f"  Grupos (g): {stats['groups']}")
    print(f"  Objetos (o): {stats['objects']}")
    
    if mtl_libs:
        print(f"\n📁 ARQUIVOS MTL REFERENCIADOS:")
        for mtl in mtl_libs:
            print(f"  - {mtl}")
    
    if materials_used:
        print(f"\n🎨 MATERIAIS USADOS ({len(materials_used)}):")
        for mat in materials_used[:10]:  # Primeiros 10
            print(f"  - {mat}")
        if len(materials_used) > 10:
            print(f"  ... e mais {len(materials_used) - 10} materiais")
    
    # Grupos únicos e análise
    unique_groups = list(set(groups))
    if unique_groups:
        print(f"\n🏗️ GRUPOS ÚNICOS ({len(unique_groups)}):")
        
        # Tentar identificar paredes
        wall_groups = []
        floor_groups = []
        other_groups = []
        
        for g in unique_groups:
            g_lower = g.lower()
            if any(word in g_lower for word in ['wall', 'parede', 'muro']):
                wall_groups.append(g)
            elif any(word in g_lower for word in ['floor', 'piso', 'chao']):
                floor_groups.append(g)
            else:
                other_groups.append(g)
        
        if wall_groups:
            print(f"\n  🧱 POSSÍVEIS PAREDES ({len(wall_groups)}):")
            for w in sorted(wall_groups)[:10]:
                faces = group_face_count.get(w, 0)
                print(f"    - {w} ({faces} faces)")
                
        if floor_groups:
            print(f"\n  🏗️ POSSÍVEIS PISOS ({len(floor_groups)}):")
            for f in sorted(floor_groups)[:10]:
                faces = group_face_count.get(f, 0)
                print(f"    - {f} ({faces} faces)")
                
        if other_groups:
            print(f"\n  📦 OUTROS GRUPOS ({len(other_groups)}):")
            # Ordenar por número de faces
            sorted_groups = sorted(other_groups, 
                                 key=lambda x: group_face_count.get(x, 0), 
                                 reverse=True)
            for g in sorted_groups[:20]:
                faces = group_face_count.get(g, 0)
                if faces > 100:  # Apenas grupos significativos
                    print(f"    - {g} ({faces} faces)")
    
    # Objetos únicos
    unique_objects = list(set(objects))
    if unique_objects:
        print(f"\n📐 OBJETOS ÚNICOS ({len(unique_objects)}):")
        for o in sorted(unique_objects)[:20]:
            print(f"  - {o}")
    
    return {
        'stats': dict(stats),
        'groups': unique_groups,
        'objects': unique_objects,
        'materials': materials_used,
        'mtl_files': mtl_libs,
        'group_faces': dict(group_face_count)
    }

if __name__ == "__main__":
    obj_file = Path("/home/pedro_setubal/Workspaces/G1/3d escritorio/escritorio_CW_scan.obj")
    
    if obj_file.exists():
        result = analyze_obj_structure(obj_file)
        
        print("\n" + "="*80)
        print("🎯 PRÓXIMOS PASSOS SUGERIDOS:")
        print("1. Instalar dependências necessárias (trimesh, obj2mjcf, coacd)")
        print("2. Identificar e extrair grupos de paredes")
        print("3. Converter paredes para MJCF")
        print("4. Testar no simulador MuJoCo")
    else:
        print(f"❌ Arquivo não encontrado: {obj_file}")