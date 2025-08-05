#!/usr/bin/env python3
"""
Script inteligente para ajustar escala de OBJ baseado na altura desejada
Calcula automaticamente o fator de escala para atingir altura específica
"""

import os
import sys

def calculate_smart_scale(input_file, target_height=3.15):
    """
    Calcula fator de escala necessário para atingir altura desejada
    
    Args:
        input_file: Arquivo OBJ a analisar
        target_height: Altura desejada em metros (default: 3.15m)
    
    Returns:
        scale_factor: Fator de escala calculado
        current_height: Altura atual do modelo
    """
    
    print(f"🔍 Analisando arquivo para calcular escala inteligente...")
    print(f"🎯 Altura desejada: {target_height}m")
    
    min_z = float('inf')
    max_z = float('-inf')
    vertex_count = 0
    
    try:
        with open(input_file, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            z = float(parts[3])  # Coordenada Z (altura)
                            min_z = min(min_z, z)
                            max_z = max(max_z, z)
                            vertex_count += 1
                        except ValueError:
                            continue
        
        if vertex_count > 0:
            current_height = max_z - min_z
            scale_factor = target_height / current_height
            
            print(f"📐 Análise atual:")
            print(f"   - Altura atual: {current_height:.2f} unidades")
            print(f"   - Min Z: {min_z:.2f}")
            print(f"   - Max Z: {max_z:.2f}")
            print(f"   - Vértices: {vertex_count}")
            print(f"🧮 Fator de escala calculado: {scale_factor:.6f}")
            print(f"✅ Resultado esperado: {current_height * scale_factor:.2f}m")
            
            return scale_factor, current_height
        else:
            print("❌ Nenhum vértice encontrado")
            return None, None
            
    except Exception as e:
        print(f"❌ Erro: {e}")
        return None, None

def apply_smart_scale(input_file, output_file, scale_factor):
    """Aplica escala inteligente calculada"""
    
    print(f"\n🔧 Aplicando escala inteligente...")
    print(f"📂 Input: {input_file}")
    print(f"💾 Output: {output_file}")
    print(f"📏 Scale factor: {scale_factor:.6f}")
    
    vertex_count = 0
    line_count = 0
    
    try:
        with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
            for line in infile:
                line_count += 1
                
                if line.startswith('v '):
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            x = float(parts[1]) * scale_factor
                            y = float(parts[2]) * scale_factor  
                            z = float(parts[3]) * scale_factor
                            
                            new_line = f"v {x:.6f} {y:.6f} {z:.6f}"
                            
                            if len(parts) > 4:
                                w = float(parts[4])
                                new_line += f" {w:.6f}"
                            
                            outfile.write(new_line + '\n')
                            vertex_count += 1
                            
                        except ValueError as e:
                            outfile.write(line)
                    else:
                        outfile.write(line)
                else:
                    outfile.write(line)
                
                if line_count % 50000 == 0:
                    print(f"📊 Processadas {line_count} linhas, {vertex_count} vértices...")
        
        print(f"✅ Escala aplicada com sucesso!")
        print(f"📊 Total: {line_count} linhas, {vertex_count} vértices processados")
        return True
        
    except Exception as e:
        print(f"❌ Erro ao aplicar escala: {e}")
        return False

def verify_result(output_file, expected_height):
    """Verifica se o resultado final está correto"""
    
    print(f"\n🔍 Verificando resultado final...")
    
    min_z = float('inf')
    max_z = float('-inf')
    vertex_count = 0
    
    try:
        with open(output_file, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            z = float(parts[3])
                            min_z = min(min_z, z)
                            max_z = max(max_z, z)
                            vertex_count += 1
                        except ValueError:
                            continue
        
        if vertex_count > 0:
            final_height = max_z - min_z
            error = abs(final_height - expected_height)
            error_percent = (error / expected_height) * 100
            
            print(f"📐 Verificação final:")
            print(f"   - Altura obtida: {final_height:.3f}m")
            print(f"   - Altura esperada: {expected_height:.3f}m")
            print(f"   - Erro: {error:.3f}m ({error_percent:.1f}%)")
            
            if error_percent < 1.0:
                print(f"✅ SUCESSO! Escala correta aplicada")
                return True
            else:
                print(f"⚠️ Erro maior que 1% - pode precisar ajuste")
                return False
        else:
            print(f"❌ Erro na verificação")
            return False
            
    except Exception as e:
        print(f"❌ Erro na verificação: {e}")
        return False

if __name__ == "__main__":
    # Configuração
    script_dir = os.path.dirname(__file__)
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
    input_file = os.path.join(base_dir, "3d escritorio", "escritorio_CW_scan.obj")
    output_file = os.path.join(base_dir, "3d escritorio", "escritorio_CW_scan_smart_scaled.obj")
    
    target_height = 3.15  # Altura desejada em metros
    
    print("🏢 Script de Escala Inteligente - Ambiente Escritório")
    print("=" * 65)
    print(f"🎯 META: Altura final = {target_height}m")
    
    # Verificar arquivo
    if not os.path.exists(input_file):
        print(f"❌ Arquivo não encontrado: {input_file}")
        sys.exit(1)
    
    # Calcular escala inteligente
    scale_factor, current_height = calculate_smart_scale(input_file, target_height)
    
    if scale_factor is None:
        print("❌ Falha no cálculo da escala")
        sys.exit(1)
    
    # Aplicar escala
    success = apply_smart_scale(input_file, output_file, scale_factor)
    
    if not success:
        print("❌ Falha na aplicação da escala")
        sys.exit(1)
    
    # Verificar resultado
    verification_ok = verify_result(output_file, target_height)
    
    print(f"\n🎉 PROCESSO CONCLUÍDO!")
    print(f"📁 Arquivo final: {os.path.basename(output_file)}")
    print(f"📐 Altura ajustada para: {target_height}m")
    print(f"🚀 Status: {'✅ SUCESSO' if verification_ok else '⚠️ VERIFICAR'}")
    print(f"\n🔧 PRÓXIMO PASSO:")
    print(f"   Importe '{os.path.basename(output_file)}' no Gazebo")