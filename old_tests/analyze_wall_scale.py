#!/usr/bin/env python3
"""
Analisa a escala das paredes para verificar se estão corretas
"""

import trimesh
import numpy as np

def analyze_wall_scale():
    # Carregar o arquivo OBJ das paredes
    mesh = trimesh.load("3d escritorio/walls_only.obj")
    
    print("=== Análise de Escala das Paredes ===")
    print(f"Número de vértices: {len(mesh.vertices)}")
    print(f"Número de faces: {len(mesh.faces)}")
    
    # Calcular dimensões
    bounds = mesh.bounds
    dimensions = bounds[1] - bounds[0]
    
    print(f"\nDimensões (min): {bounds[0]}")
    print(f"Dimensões (max): {bounds[1]}")
    print(f"Tamanho total: {dimensions}")
    
    # Converter de mm para metros (assumindo que o scan está em mm)
    dimensions_m = dimensions / 1000.0
    print(f"\nDimensões em metros (se original em mm):")
    print(f"  Largura (X): {dimensions_m[0]:.2f}m")
    print(f"  Profundidade (Y): {dimensions_m[1]:.2f}m")  
    print(f"  Altura (Z): {dimensions_m[2]:.2f}m")
    
    # Centro da geometria
    center = mesh.centroid
    print(f"\nCentro da geometria: {center}")
    print(f"Centro em metros: {center/1000.0}")
    
    # Sugerir escala
    print("\n=== Sugestão para MuJoCo ===")
    print("Se as paredes parecem muito grandes, adicione scale='0.001 0.001 0.001' ao body")
    print("para converter de milímetros para metros")

if __name__ == "__main__":
    analyze_wall_scale()