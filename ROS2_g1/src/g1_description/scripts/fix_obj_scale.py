#!/usr/bin/env python3
"""
Script para corrigir escala de arquivo OBJ de mm para metros
Converte coordenadas do scan LiDAR de milímetros para metros (fator 0.001)
"""

import os
import sys

def fix_obj_scale(input_file, output_file, scale_factor=0.001):
    """
    Corrige escala de arquivo OBJ aplicando fator de escala aos vértices
    
    Args:
        input_file: Caminho do arquivo OBJ original
        output_file: Caminho do arquivo OBJ corrigido
        scale_factor: Fator de escala (default: 0.001 para mm→m)
    """
    
    print(f"🔧 Iniciando correção de escala...")
    print(f"📂 Input: {input_file}")
    print(f"💾 Output: {output_file}")
    print(f"📏 Scale factor: {scale_factor}")
    
    vertex_count = 0
    line_count = 0
    
    try:
        with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
            for line in infile:
                line_count += 1
                
                # Processar apenas linhas de vértices (começam com 'v ')
                if line.startswith('v '):
                    parts = line.strip().split()
                    if len(parts) >= 4:  # 'v' + x + y + z (+ opcional w)
                        try:
                            # Aplicar escala às coordenadas x, y, z
                            x = float(parts[1]) * scale_factor
                            y = float(parts[2]) * scale_factor  
                            z = float(parts[3]) * scale_factor
                            
                            # Reescrever linha com coordenadas corrigidas
                            new_line = f"v {x:.6f} {y:.6f} {z:.6f}"
                            
                            # Manter coordenada w se existir
                            if len(parts) > 4:
                                w = float(parts[4])
                                new_line += f" {w:.6f}"
                            
                            outfile.write(new_line + '\n')
                            vertex_count += 1
                            
                        except ValueError as e:
                            print(f"⚠️ Erro ao processar vértice linha {line_count}: {e}")
                            outfile.write(line)  # Manter linha original se erro
                    else:
                        outfile.write(line)  # Manter linha original se formato inválido
                        
                else:
                    # Copiar todas as outras linhas sem modificação
                    outfile.write(line)
                
                # Progress indicator
                if line_count % 10000 == 0:
                    print(f"📊 Processadas {line_count} linhas, {vertex_count} vértices...")
    
    except FileNotFoundError:
        print(f"❌ Erro: Arquivo não encontrado: {input_file}")
        return False
    except Exception as e:
        print(f"❌ Erro inesperado: {e}")
        return False
    
    print(f"✅ Conversão concluída!")
    print(f"📊 Estatísticas:")
    print(f"   - Linhas processadas: {line_count}")
    print(f"   - Vértices corrigidos: {vertex_count}")
    print(f"   - Fator aplicado: {scale_factor}")
    print(f"💾 Arquivo salvo em: {output_file}")
    
    return True

def analyze_obj_bounds(obj_file):
    """Analisa dimensões do arquivo OBJ para verificar escala"""
    
    print(f"🔍 Analisando dimensões do arquivo: {obj_file}")
    
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    vertex_count = 0
    
    try:
        with open(obj_file, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            
                            min_x, max_x = min(min_x, x), max(min_x, x)
                            min_y, max_y = min(min_y, y), max(max_y, y)  
                            min_z, max_z = min(min_z, z), max(max_z, z)
                            
                            vertex_count += 1
                            
                        except ValueError:
                            continue
        
        if vertex_count > 0:
            dim_x = max_x - min_x
            dim_y = max_y - min_y
            dim_z = max_z - min_z
            
            print(f"📐 Dimensões encontradas:")
            print(f"   - X: {dim_x:.2f} ({min_x:.2f} a {max_x:.2f})")
            print(f"   - Y: {dim_y:.2f} ({min_y:.2f} a {max_y:.2f})")
            print(f"   - Z: {dim_z:.2f} ({min_z:.2f} a {max_z:.2f})")
            print(f"   - Maior dimensão: {max(dim_x, dim_y, dim_z):.2f}")
            print(f"   - Menor dimensão: {min(dim_x, dim_y, dim_z):.2f}")
            print(f"🧮 Total vértices: {vertex_count}")
            
            return dim_x, dim_y, dim_z
        else:
            print("❌ Nenhum vértice encontrado no arquivo")
            return None
            
    except Exception as e:
        print(f"❌ Erro ao analisar arquivo: {e}")
        return None

if __name__ == "__main__":
    # Configuração dos caminhos - corrigido para estrutura real
    script_dir = os.path.dirname(__file__)  # /ROS2_g1/src/g1_description/scripts/
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))  # /Workspaces/G1/
    input_file = os.path.join(base_dir, "3d escritorio", "escritorio_CW_scan.obj")
    output_file = os.path.join(base_dir, "3d escritorio", "escritorio_CW_scan_scaled.obj")
    
    print("🏢 Script de Correção de Escala - Ambiente Escritório")
    print("=" * 60)
    
    # Verificar se arquivo existe
    if not os.path.exists(input_file):
        print(f"❌ Arquivo não encontrado: {input_file}")
        sys.exit(1)
    
    # Analisar dimensões originais
    print("\n📊 ANÁLISE ORIGINAL:")
    original_dims = analyze_obj_bounds(input_file)
    
    if original_dims:
        dim_x, dim_y, dim_z = original_dims
        max_dim = max(dim_x, dim_y, dim_z)
        
        print(f"\n🎯 OBJETIVO: Converter de ~{max_dim:.0f}mm para ~{max_dim*0.001:.1f}m")
        
        # Aplicar correção
        print(f"\n🔧 APLICANDO CORREÇÃO DE ESCALA:")
        success = fix_obj_scale(input_file, output_file, scale_factor=0.001)
        
        if success:
            print(f"\n📊 ANÁLISE CORRIGIDA:")
            analyze_obj_bounds(output_file)
            
            print(f"\n🎉 SUCESSO! Arquivo corrigido salvo em:")
            print(f"   {output_file}")
            print(f"\n🚀 PRÓXIMO PASSO:")
            print(f"   Importe {os.path.basename(output_file)} no Gazebo")
        else:
            print(f"\n❌ FALHA na correção de escala")
            sys.exit(1)
    else:
        print(f"\n❌ FALHA na análise do arquivo original")
        sys.exit(1)