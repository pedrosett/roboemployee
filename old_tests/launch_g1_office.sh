#!/bin/bash
cd ~/Workspaces/G1

echo "=== Instruções para visualizar G1 no escritório ==="
echo ""
echo "1. Primeiro, execute este comando para ver o G1 completo:"
echo "   simulate unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml"
echo ""
echo "2. Depois, execute este comando para ver as paredes do escritório:"
echo "   simulate g1_office_working.xml"
echo ""
echo "3. Para carregar ambos juntos, precisamos combinar os arquivos XML manualmente"
echo ""
echo "Executando as paredes do escritório agora..."
echo ""

simulate g1_office_working.xml