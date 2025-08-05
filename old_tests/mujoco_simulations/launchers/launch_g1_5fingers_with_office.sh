#!/bin/bash
# Lançador G1 5 Dedos + Ambiente de Escritório

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║    G1EDU U6 (5 DEDOS) + AMBIENTE DE ESCRITÓRIO - MODO DUAL   ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "🤖 Robô: G1Edu U6 com mãos RH56DFQ de 5 dedos"
echo "🏢 Ambiente: Escritório com paredes escaneadas"
echo ""
echo "🎯 OPÇÕES DISPONÍVEIS:"
echo ""
echo "1️⃣  G1 com mãos de 5 dedos (modelo correto do invoice)"
echo "2️⃣  Ambiente de escritório com paredes"
echo "3️⃣  Comparar modelos (3 dedos vs 5 dedos)"
echo ""
echo -n "Escolha uma opção (1-3): "
read option

case $option in
    1)
        echo ""
        echo "🚀 Carregando G1Edu U6 com mãos de 5 dedos..."
        echo "✋ RH56DFQ-2R/2L - 17 sensores táteis por mão"
        echo ""
        cd ~/Workspaces/G1/unitree_ros/robots/g1_description
        simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
        ;;
    2)
        echo ""
        echo "🏢 Carregando ambiente de escritório com G1 básico..."
        echo "📐 Paredes 33m x 28m x 3.15m de altura"
        echo ""
        cd ~/Workspaces/G1
        simulate g1_office_unified.xml
        ;;
    3)
        echo ""
        echo "🔍 Comparação de modelos:"
        echo ""
        echo "📋 PRIMEIRO: G1 com mãos de 3 dedos (modelo antigo)"
        echo "Pressione ENTER para continuar..."
        read
        cd ~/Workspaces/G1
        simulate g1_office_unified.xml &
        PID1=$!
        
        echo ""
        echo "📋 SEGUNDO: G1 com mãos de 5 dedos (modelo correto)"
        echo "Pressione ENTER para continuar..."
        read
        cd ~/Workspaces/G1/unitree_ros/robots/g1_description
        simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf &
        PID2=$!
        
        echo "✅ Ambos carregados! Compare as mãos:"
        echo "   • Janela 1: 3 dedos (Dex3-1)"
        echo "   • Janela 2: 5 dedos (RH56DFQ)"
        echo ""
        echo "Pressione ENTER para fechar..."
        read
        kill $PID1 $PID2 2>/dev/null
        ;;
    *)
        echo "❌ Opção inválida!"
        exit 1
        ;;
esac

echo ""
echo "✅ Simulação concluída!"