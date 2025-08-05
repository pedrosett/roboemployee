# Simulação Unitree G1 em Ambiente de Escritório 3D

Simulação completa do robô humanoide **Unitree G1** em ambiente de escritório real escaneado via LiDAR, usando MuJoCo 3.2.7.

## 🚀 Execução Rápida

```bash
cd ~/Workspaces/G1
simulate g1_office_unified.xml
```

Ou use o lançador:
```bash
./launch_simulation.sh
```

## 📊 Especificações

- **Robô**: Unitree G1 (1.2m altura, 29 DOF + mãos Dex3-1)
- **Ambiente**: Escritório 33m x 28m x 3.15m
- **Simulador**: MuJoCo 3.2.7
- **Física**: Totalmente funcional

## 📁 Estrutura do Projeto

```
G1/
├── g1_office_unified.xml      # Arquivo principal da simulação
├── launch_simulation.sh       # Script de lançamento
├── g1_office_simulation/      # Documentação detalhada
├── 3d escritorio/            # Assets 3D do escritório
├── unitree_mujoco/           # Modelos do robô G1
└── old_tests/                # Arquivos de desenvolvimento (arquivado)
```

## 📖 Documentação

- **Documentação completa**: `g1_office_simulation/README.md`
- **Guia rápido**: `g1_office_simulation/QUICK_START.md`
- **Status do projeto**: `PROJECT_STATUS.md`

## ✅ Status

**PROJETO CONCLUÍDO** - Simulação pronta para desenvolvimento de algoritmos de controle robótico.

---
Criado em 04/08/2024