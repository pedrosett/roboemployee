#!/bin/bash

echo "🤖 Testando G1 com mãos completas diretamente no MuJoCo"
echo "======================================================"

G1_MODEL="/home/pedro_setubal/Workspaces/G1/unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml"

if [ ! -f "$G1_MODEL" ]; then
    echo "❌ Modelo G1 não encontrado: $G1_MODEL"
    exit 1
fi

echo "✅ Modelo encontrado: $G1_MODEL"
echo ""
echo "🎮 Controles do MuJoCo:"
echo "  Mouse: Rotacionar câmera"
echo "  Shift+Mouse: Pan"
echo "  Scroll: Zoom"
echo "  Ctrl+L: Carregar modelo"
echo "  Space: Pausar/continuar"
echo "  ESC: Sair"
echo ""
echo "🚀 Executando simulador MuJoCo com G1..."
echo ""

# Executar simulador MuJoCo padrão
simulate "$G1_MODEL"