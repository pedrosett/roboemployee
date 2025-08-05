# 🚀 Guia Rápido - Simulação G1 no Escritório

## Executar a Simulação

```bash
cd ~/Workspaces/G1
simulate g1_office_unified.xml
```

## Controles do Simulador

- **Mouse**: Rotacionar câmera (arrastar)
- **Shift + Mouse**: Mover câmera (pan)
- **Scroll**: Zoom
- **Espaço**: Pausar/continuar simulação
- **Backspace**: Reset
- **Ctrl+S**: Salvar screenshot

## Arquivos Principais

1. **g1_office_unified.xml** - Arquivo principal (executar este!)
2. **3d escritorio/body1_structure/** - Paredes do escritório
3. **unitree_mujoco/unitree_robots/g1/** - Modelo do robô

## Especificações

- **Robô G1**: 1.2m altura, 29 DOF + mãos
- **Escritório**: 33m x 28m x 3.15m
- **Escala**: Metros (convertido de mm com scale=0.01)

---
Para documentação completa, veja README.md