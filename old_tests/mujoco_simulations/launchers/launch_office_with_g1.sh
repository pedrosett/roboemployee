#!/bin/bash
# Lançador Direto: G1 no Ambiente de Escritório

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         G1 + AMBIENTE DE ESCRITÓRIO - SIMULAÇÃO DIRETA       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "🤖 Robô: Unitree G1 (29 DOF + mãos de 3 dedos)"
echo "🏢 Ambiente: Escritório 33m x 28m x 3.15m"
echo "📐 Escala: Robô 1.2m vs Paredes 3.15m"
echo ""
echo "🎮 CONTROLES:"
echo "   ESPAÇO     - Pausar/continuar"
echo "   SETA →     - Avançar um passo"
echo "   Backspace  - Resetar"
echo "   Mouse      - Controlar câmera"
echo ""
echo "🚀 Iniciando simulação G1 + escritório..."
echo ""

cd ~/Workspaces/G1
simulate g1_office_unified.xml

echo "✅ Simulação concluída!"