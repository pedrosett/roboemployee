#!/bin/bash

# Script para executar simulação do G1 com mãos completas

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SIM_DIR="$SCRIPT_DIR/unitree_mujoco/simulate"
BUILD_DIR="$SIM_DIR/build"

echo "🤖 Unitree G1 - Simulação com Mãos Completas (29 DOF)"
echo "====================================================="

# Verificar se simulador existe
if [ ! -f "$BUILD_DIR/unitree_mujoco" ]; then
    echo "❌ Erro: Simulador não encontrado!"
    echo "   Compile primeiro seguindo as instruções no README.md"
    exit 1
fi

# Criar configuração se não existir
CONFIG_FILE="$SIM_DIR/config_g1_hands.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "📝 Criando configuração..."
    cat > "$CONFIG_FILE" << EOF
robot: "g1"
robot_scene: "scene_29dof_with_hand.xml"
domain_id: 1
interface: "lo"
use_joystick: 0
joystick_type: "xbox"
joystick_device: "/dev/input/js0"
joystick_bits: 16
print_scene_information: 1
enable_elastic_band: 1
EOF
    echo "✅ Configuração criada: $CONFIG_FILE"
fi

echo ""
echo "🎮 Controles:"
echo "  Mouse: Rotacionar câmera"
echo "  Shift+Mouse: Pan"
echo "  Scroll: Zoom"
echo "  Tecla '9': Ativar/desativar elástico"
echo "  Tecla '8': Levantar robô"
echo "  Tecla '7': Abaixar robô"
echo "  ESC: Sair"
echo ""
echo "📊 O que verificar:"
echo "  - Deve mostrar 29 joints no log"
echo "  - Juntas das mãos devem estar listadas (índices 23-28)"
echo "  - O robô deve estar suspenso pelo elástico virtual"
echo ""
echo "🚀 Iniciando simulação..."
echo ""

cd "$BUILD_DIR"
./unitree_mujoco -c "../config_g1_hands.yaml"