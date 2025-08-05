#!/usr/bin/env python3
"""
Script para analisar o arquivo OBJ do escritório e identificar sua estrutura
"""

import trimesh
import numpy as np
from pathlib import Path
import re
from collections import defaultdict

def analyze_obj_file(obj_path):
    """Analisa arquivo OBJ e extrai informações sobre grupos e objetos"""
    print(f"Analisando arquivo: {obj_path}")
    print("-" * 80)
    
    # Estatísticas básicas
    with open(obj_path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
    
    # Contadores
    stats = defaultdict(int)
    groups = []
    objects = []
    materials = []
    current_group = None
    current_object = None
    
    # Padrões regex
    vertex_pattern = re.compile(r'^v\s+')
    face_pattern = re.compile(r'^f\s+')
    group_pattern = re.compile(r'^g\s+(.+)')
    object_pattern = re.compile(r'^o\s+(.+)')
    mtllib_pattern = re.compile(r'^mtllib\s+(.+)')
    usemtl_pattern = re.compile(r'^usemtl\s+(.+)')
    
    print("1. ANÁLISE LINHA POR LINHA:")
    for line in lines[:50]:  # Primeiras 50 linhas para debug
        line = line.strip()
        if vertex_pattern.match(line):
            stats['vertices'] += 1
        elif face_pattern.match(line):
            stats['faces'] += 1
        elif group_match := group_pattern.match(line):
            group_name = group_match.group(1)
            groups.append(group_name)
            current_group = group_name
            if stats['vertices'] < 10:  # Mostra apenas os primeiros
                print(f"  Grupo encontrado: {group_name}")
        elif object_match := object_pattern.match(line):
            object_name = object_match.group(1)
            objects.append(object_name)
            current_object = object_name
            if stats['vertices'] < 10:
                print(f"  Objeto encontrado: {object_name}")
        elif mtllib_match := mtllib_pattern.match(line):
            mtl_file = mtllib_match.group(1)
            print(f"  MTL referenciado: {mtl_file}")
        elif usemtl_match := usemtl_pattern.match(line):
            material = usemtl_match.group(1)
            if material not in materials:
                materials.append(material)
    
    print(f"\n2. ESTATÍSTICAS BÁSICAS:")
    print(f"  Total de linhas: {len(lines)}")
    print(f"  Vértices (v): {stats['vertices']}")
    print(f"  Faces (f): {stats['faces']}")
    print(f"  Grupos (g): {len(groups)}")
    print(f"  Objetos (o): {len(objects)}")
    print(f"  Materiais usados: {len(materials)}")
    
    # Usar trimesh para análise mais detalhada
    print(f"\n3. ANÁLISE COM TRIMESH:")
    try:
        # Carregar com diferentes opções
        mesh = trimesh.load(
            obj_path,
            split_object=True,
            group_material=True,
            process=False,
            maintain_order=True
        )
        
        if isinstance(mesh, trimesh.Scene):
            print(f"  Arquivo carregado como Scene com {len(mesh.geometry)} geometrias")
            print("\n  GEOMETRIAS ENCONTRADAS:")
            for i, (name, geom) in enumerate(mesh.geometry.items()):
                bounds = geom.bounds
                size = bounds[1] - bounds[0]
                print(f"    [{i}] {name}:")
                print(f"        Vértices: {len(geom.vertices)}")
                print(f"        Faces: {len(geom.faces)}")
                print(f"        Dimensões (m): X={size[0]:.2f}, Y={size[1]:.2f}, Z={size[2]:.2f}")
                print(f"        Centro: {geom.centroid}")
                
                # Tentar identificar tipo pelo nome ou tamanho
                if 'wall' in name.lower() or 'parede' in name.lower():
                    print(f"        → Possível PAREDE")
                elif 'floor' in name.lower() or 'piso' in name.lower():
                    print(f"        → Possível PISO")
                elif size[2] > 2.0:  # Altura > 2m
                    print(f"        → Possível PAREDE (pela altura)")
                
        else:
            print(f"  Arquivo carregado como mesh única")
            print(f"  Vértices: {len(mesh.vertices)}")
            print(f"  Faces: {len(mesh.faces)}")
            bounds = mesh.bounds
            size = bounds[1] - bounds[0]
            print(f"  Dimensões totais (m): X={size[0]:.2f}, Y={size[1]:.2f}, Z={size[2]:.2f}")
            
    except Exception as e:
        print(f"  Erro ao carregar com trimesh: {e}")
    
    # Listar grupos e objetos únicos
    if groups:
        print(f"\n4. GRUPOS ÚNICOS ({len(set(groups))}):")
        for g in sorted(set(groups))[:20]:  # Primeiros 20
            print(f"  - {g}")
            
    if objects:
        print(f"\n5. OBJETOS ÚNICOS ({len(set(objects))}):")
        for o in sorted(set(objects))[:20]:  # Primeiros 20
            print(f"  - {o}")
    
    return {
        'groups': groups,
        'objects': objects,
        'materials': materials,
        'stats': dict(stats)
    }

if __name__ == "__main__":
    obj_file = Path("/home/pedro_setubal/Workspaces/G1/3d escritorio/escritorio_CW_scan.obj")
    if obj_file.exists():
        analyze_obj_file(obj_file)
    else:
        print(f"Arquivo não encontrado: {obj_file}")