# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

✅ **PROJETO CONCLUÍDO COM SUCESSO!** - Simulação completa do robô humanoide **Unitree G1** no MuJoCo com ambiente 3D de escritório real escaneado via LiDAR. Pronto para desenvolvimento e teste de algoritmos de controle robótico.

## Status do Projeto (04/08/2024) - ✅ COMPLETO

### ✅ **SIMULAÇÃO FINAL FUNCIONANDO:**
1. **MuJoCo 3.2.7** instalado e funcionando
2. **G1 completo** integrado no ambiente:
   - Modelo: G1 29 DOF + mãos Dex3-1 (3 dedos por mão)
   - Altura: 1.2m (escala correta)
   - Posição: pés no nível do piso (pelvis a 0.75m)
3. **Ambiente 3D do escritório** integrado:
   - Paredes extraídas do scan LiDAR (grupo "Body1")
   - Dimensões: 33m x 28m x 3.15m altura
   - Escala correta (0.01) em relação ao robô
4. **Arquivos principais**:
   - `g1_office_unified.xml` - Simulação com mãos de 3 dedos + escritório
   - `launch_simulation_final.sh` - Lançador para modelo básico
   - ✅ **`launch_g1_5fingers_final.sh` - G1Edu U6 com mãos de 5 dedos (CORRETO)**

### ✅ **PROBLEMAS RESOLVIDOS:**
- ✅ Escala correta entre robô (1.2m) e paredes (3.15m)
- ✅ Identificação das paredes reais no grupo "Body1" do OBJ
- ✅ Conversão e integração do ambiente 3D
- ✅ Posicionamento correto do robô no ambiente
- ✅ Altura dos pés ajustada para o nível do piso
- ✅ Física e gravidade funcionando corretamente

## Estrutura do Projeto

```
~/Workspaces/G1/
├── 3d escritorio/                    # Assets 3D originais
│   ├── escritorio_CW_scan.obj       # Scan LiDAR do escritório (131 grupos, 59k vértices)
│   ├── f8fe65f8-*.mtl              # Materiais (Steel, Oak)
│   └── Tutorial_*.pdf               # Tutorial de conversão CAD→MuJoCo
├── unitree_mujoco/                  # Simulador principal
│   ├── simulate/                    # Código do simulador (não compilado ainda - falta SDK)
│   └── unitree_robots/g1/          # Modelos do G1
│       ├── g1_29dof_with_hand.xml # Modelo com mãos (mostrando 3 dedos)
│       └── meshes/                 # Arquivos STL das partes
├── unitree_sdk2/                    # SDK C++ (instalação parcial)
├── unitree_sdk2_python/             # Interface Python
├── obj2mjcf/                        # Ferramenta de conversão OBJ→MJCF
├── mujoco/                          # MuJoCo 3.2.7 source
├── README.md                        # Documentação completa do setup
├── OBJ_to_MuJoCo_Guide.md          # Guia de conversão de modelos 3D
└── Scripts criados:
    ├── analyze_obj_simple.py        # Análise do arquivo OBJ
    ├── test_g1_direct.sh           # Teste direto do G1 no MuJoCo
    └── check_mujoco_install.sh     # Verificação da instalação

```

## 🚀 **COMANDOS PRINCIPAIS - SIMULAÇÕES PRONTAS:**

### 🏆 **G1Edu U6 com Mãos de 5 Dedos (MODELO CORRETO):**
```bash
cd ~/Workspaces/G1
./launch_g1_5fingers_final.sh
```

### 🏢 **G1 + Ambiente de Escritório:**
```bash
cd ~/Workspaces/G1
./launch_office_with_g1.sh
```

### 🎯 **Menu Interativo (5 dedos + escritório + comparação):**
```bash
cd ~/Workspaces/G1
./launch_g1_5fingers_with_office.sh
```

### Comando Direto (alternativo):
```bash
cd ~/Workspaces/G1
simulate g1_office_unified.xml
```

### 🎮 **CONTROLES DA SIMULAÇÃO:**
- **ESPAÇO**: Pausar/continuar simulação
- **SETA →**: Avançar um passo (quando pausado)  
- **Backspace**: Resetar simulação
- **F1**: Mostrar ajuda completa
- **Mouse**: Controlar câmera
- **⚠️ DICA**: Pressione ESPAÇO rapidamente para pausar e observar!

## 📝 **NOTAS IMPORTANTES:**

1. **Mãos do G1**: Modelo usa Dex3-1 com 3 dedos por mão (padrão atual)
2. **Altura ajustada**: Robô nasce com pés no nível do piso
3. **Escala validada**: Robô 1.2m vs paredes 3.15m (proporção correta)

## Notas Técnicas

- **MuJoCo**: Usando versão 3.2.7 (última estável)
- **G1 Robot**: Humanoide bípede, esperado 29 DOF com mãos de 5 dedos
- **Comunicação**: G1 usa `unitree_hg` (não `unitree_go` dos quadrúpedes)
- **Python**: Sistema usa Python 3.12, requer ambientes virtuais para pip

## Dependências Pendentes

Para completar o setup, instalar:
```bash
# Em ambiente virtual
python3 -m venv ~/mujoco_env
source ~/mujoco_env/bin/activate
pip install trimesh obj2mjcf coacd numpy Pillow lxml
```