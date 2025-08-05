#!/bin/bash
echo "=== Comparação de Escala: G1 vs Paredes do Escritório ==="
echo ""
echo "Abrindo duas janelas do simulador para comparação:"
echo ""
echo "1. Janela 1: G1 completo (altura ~1.2m)"
echo "2. Janela 2: Paredes do escritório (altura 3.15m)"
echo ""
echo "Pressione ENTER depois de visualizar o G1..."

cd ~/Workspaces/G1
simulate unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml &

read -p "Agora pressione ENTER para ver as paredes..."

simulate g1_office_working.xml &

echo ""
echo "Compare as escalas nas duas janelas."
echo "O G1 deveria ter aproximadamente 1/3 da altura das paredes."