#!/bin/bash
# Lançador Principal da Simulação G1 + Escritório

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         Simulação Unitree G1 em Ambiente de Escritório       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "🤖 Robô: Unitree G1 (1.2m, 29 DOF + mãos Dex3-1)"
echo "🏢 Ambiente: Escritório 33m x 28m x 3.15m (scan LiDAR)"
echo "⚖️  Física: Gravidade normal (-9.81 m/s²)"
echo ""
echo "🎮 CONTROLES DA SIMULAÇÃO:"
echo "   ESPAÇO     - Pausar/continuar simulação"
echo "   SETA →     - Avançar um passo (quando pausado)"
echo "   Backspace  - Resetar simulação"
echo "   F1         - Mostrar ajuda completa"
echo "   Mouse      - Controlar câmera"
echo ""
echo "⚠️  DICA: Pressione ESPAÇO rapidamente para pausar e observar!"
echo ""

cd ~/Workspaces/G1
simulate g1_office_unified.xml