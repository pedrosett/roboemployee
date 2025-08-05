#!/bin/bash

# G1 5-Finger Standalone Simulation Launcher
# Unitree G1 humanoid robot with 5-finger hands (no office environment)

echo "🤖 Lançando G1 com mãos de 5 dedos (sem escritório)..."
echo "📁 Arquivo: g1_5fingers_standalone.xml"
echo ""
echo "🎮 Controles:"
echo "  ESPAÇO: Pausar/continuar"
echo "  SETA →: Avançar passo (pausado)"  
echo "  Backspace: Resetar"
echo "  F1: Ajuda completa"
echo "  Mouse: Controlar câmera"
echo ""

cd ~/Workspaces/G1

# Verificar se o arquivo existe
if [ ! -f "g1_5fingers_standalone.xml" ]; then
    echo "❌ Erro: Arquivo g1_5fingers_standalone.xml não encontrado!"
    exit 1
fi

# Verificar se os meshes existem
if [ ! -d "unitree_ros/robots/g1_description/meshes" ]; then
    echo "❌ Erro: Diretório de meshes não encontrado!"
    echo "   Esperado: unitree_ros/robots/g1_description/meshes"
    exit 1
fi

echo "🚀 Iniciando simulação MuJoCo..."
simulate g1_5fingers_standalone.xml