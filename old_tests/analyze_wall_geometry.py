#!/usr/bin/env python3
"""
Analisa a geometria do grupo de paredes para entender melhor sua estrutura
"""

import trimesh
import numpy as np

def analyze_wall_geometry():
    # Carregar o arquivo OBJ das paredes
    mesh = trimesh.load("3d escritorio/walls_only.obj")
    
    print("=== Análise Detalhada da Geometria ===")
    
    # Se for uma cena com múltiplos objetos
    if hasattr(mesh, 'geometry'):
        print(f"Número de geometrias na cena: {len(mesh.geometry)}")
        for name, geom in mesh.geometry.items():
            print(f"\nGeometria: {name}")
            print(f"  Vértices: {len(geom.vertices)}")
            print(f"  Faces: {len(geom.faces)}")
            bounds = geom.bounds
            dims = bounds[1] - bounds[0]
            print(f"  Dimensões: {dims}")
            print(f"  Dimensões em metros: {dims/1000.0}")
    else:
        # É uma única mesh
        print("Mesh única encontrada")
        print(f"Vértices: {len(mesh.vertices)}")
        print(f"Faces: {len(mesh.faces)}")
        
    # Analisar a distribuição dos vértices
    vertices = mesh.vertices if not hasattr(mesh, 'geometry') else list(mesh.geometry.values())[0].vertices
    
    # Análise por eixo
    print("\n=== Análise por Eixo ===")
    for i, axis in enumerate(['X', 'Y', 'Z']):
        values = vertices[:, i]
        print(f"\nEixo {axis}:")
        print(f"  Min: {np.min(values):.2f}")
        print(f"  Max: {np.max(values):.2f}")
        print(f"  Média: {np.mean(values):.2f}")
        print(f"  Range: {np.max(values) - np.min(values):.2f}")
        
    # Identificar possíveis planos (paredes)
    print("\n=== Análise de Planos ===")
    
    # Agrupar vértices por valores X similares (paredes perpendiculares a X)
    x_values = vertices[:, 0]
    unique_x = np.unique(np.round(x_values, decimals=0))
    print(f"\nValores X únicos (arredondados): {len(unique_x)}")
    if len(unique_x) < 20:
        print("Valores:", unique_x)
    
    # Agrupar vértices por valores Y similares (paredes perpendiculares a Y)
    y_values = vertices[:, 1]
    unique_y = np.unique(np.round(y_values, decimals=0))
    print(f"\nValores Y únicos (arredondados): {len(unique_y)}")
    if len(unique_y) < 20:
        print("Valores:", unique_y)
        
    # Visualizar distribuição de altura (Z)
    z_values = vertices[:, 2]
    print(f"\nAltura (Z):")
    print(f"  Mínima: {np.min(z_values):.2f}mm ({np.min(z_values)/1000:.2f}m)")
    print(f"  Máxima: {np.max(z_values):.2f}mm ({np.max(z_values)/1000:.2f}m)")
    
    # Salvar análise visual
    print("\n=== Salvando visualização ===")
    
    # Criar uma versão com cores por altura
    colors = np.zeros((len(vertices), 4))
    z_normalized = (z_values - np.min(z_values)) / (np.max(z_values) - np.min(z_values))
    colors[:, 0] = z_normalized  # Vermelho aumenta com altura
    colors[:, 2] = 1 - z_normalized  # Azul diminui com altura
    colors[:, 3] = 1.0  # Alpha
    
    if not hasattr(mesh, 'geometry'):
        mesh.visual.vertex_colors = colors
        
    # Exportar com escala correta
    mesh_scaled = mesh.copy()
    mesh_scaled.apply_scale(0.001)  # Converter para metros
    mesh_scaled.export("3d escritorio/walls_scaled_meters.obj")
    print("Arquivo salvo: 3d escritorio/walls_scaled_meters.obj (em metros)")

if __name__ == "__main__":
    analyze_wall_geometry()