#!/bin/bash
# Script para converter URDF do G1 de 5 dedos para MJCF

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║          CONVERSÃO URDF → MJCF (XML do MuJoCo)              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "📋 INSTRUÇÕES:"
echo ""
echo "1️⃣  O MuJoCo Viewer abrirá com o robô G1 de 5 dedos"
echo ""
echo "2️⃣  No menu, clique em: File → Save xml"
echo ""
echo "3️⃣  Salve como: g1_5fingers_converted.xml"
echo ""
echo "4️⃣  Feche o viewer quando terminar"
echo ""
echo "🚀 Abrindo o MuJoCo Viewer..."
echo ""

cd ~/Workspaces/G1/unitree_ros/robots/g1_description
simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf

echo ""
echo "✅ Conversão concluída!"
echo ""
echo "📁 Próximos passos:"
echo "   1. Copie o arquivo XML convertido para o diretório principal"
echo "   2. Crie um XML principal que combine robô + ambiente"
echo ""