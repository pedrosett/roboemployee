#!/bin/bash
# Lançador da Simulação G1 + Escritório com Auto-Pause

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         Simulação Unitree G1 em Ambiente de Escritório       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "🤖 Robô: Unitree G1 (1.2m, 29 DOF + mãos)"
echo "🏢 Ambiente: Escritório 33m x 28m x 3.15m"
echo ""
echo "🎮 CONTROLES DA SIMULAÇÃO:"
echo "   ESPAÇO     - Pausar/continuar simulação"
echo "   SETA →     - Avançar um passo (quando pausado)"
echo "   Backspace  - Resetar simulação"
echo "   F1         - Mostrar ajuda completa"
echo "   Mouse      - Controlar câmera"
echo ""
echo "🤖 INICIANDO SIMULAÇÃO JÁ PAUSADA..."
echo "   A simulação será pausada automaticamente após 1 segundo"
echo "   Pressione ESPAÇO para despausar quando quiser observar"
echo ""

cd ~/Workspaces/G1

# Inicia a simulação em background
simulate g1_office_unified.xml &
SIMULATE_PID=$!

# Aguarda 1 segundo para a janela carregar
sleep 1

# Encontra a janela do MuJoCo e envia ESPAÇO para pausar
WINDOW_ID=$(xdotool search --name "MuJoCo" | head -1)

if [ ! -z "$WINDOW_ID" ]; then
    echo "✅ Pausando simulação automaticamente..."
    xdotool windowactivate $WINDOW_ID
    sleep 0.2
    xdotool key space
    echo "✅ Simulação pausada! Pressione ESPAÇO na janela para continuar."
else
    echo "⚠️  Não foi possível pausar automaticamente."
    echo "   Pressione ESPAÇO rapidamente na janela da simulação!"
fi

# Aguarda o processo da simulação terminar
wait $SIMULATE_PID