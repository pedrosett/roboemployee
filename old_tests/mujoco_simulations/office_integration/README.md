# Simulação Unitree G1 em Ambiente de Escritório 3D

## 🎯 Resultado Final

Conseguimos criar com sucesso uma simulação completa do robô humanoide **Unitree G1** em um ambiente de escritório realista escaneado via LiDAR, com física funcional e escalas corretas no simulador MuJoCo.

### Características da Simulação:
- **Robô**: Unitree G1 com 29 DOF e mãos Dex3-1 (3 dedos por mão)
- **Ambiente**: Escritório de 33m x 28m com paredes de 3.15m de altura
- **Física**: Totalmente funcional com gravidade, colisões e articulações
- **Escala**: Proporções corretas (G1 ~1.2m, paredes 3.15m)

## 📁 Estrutura do Projeto

```
g1_office_simulation/
├── g1_office_unified.xml      # Arquivo principal da simulação
├── scripts/                   # Scripts Python utilizados
│   ├── create_unified_scene.py
│   ├── extract_body1.py
│   └── analyze_body_groups.py
├── docs/                      # Documentação
│   └── README.md (este arquivo)
└── assets/                    # Links para assets necessários
```

## 🚀 Como Executar

### Pré-requisitos
1. MuJoCo 3.2.7 instalado
2. Python 3.12 com ambiente virtual
3. Dependências Python: trimesh, obj2mjcf, coacd

### Comando de Execução
```bash
cd ~/Workspaces/G1
simulate g1_office_unified.xml
```

## 🔧 Processo de Desenvolvimento

### 1. Análise do Arquivo OBJ Original
O arquivo `escritorio_CW_scan.obj` continha 131 grupos diferentes. Descobrimos que:
- O grupo "mesa parede tvs" era apenas um móvel pequeno
- O grupo **"Body1"** continha as paredes reais do escritório
- Dimensões originais: 3283mm x 2836mm x 315mm

### 2. Extração e Conversão das Paredes
```python
# Script: extract_body1.py
# Extrai o grupo Body1 do arquivo OBJ original
python extract_body1.py

# Conversão para MuJoCo usando obj2mjcf
obj2mjcf --obj-dir "3d escritorio" --obj-filter "body1_structure" \
         --save-mjcf --decompose --verbose --overwrite
```

### 3. Descoberta da Escala Correta
- Arquivo OBJ estava em milímetros
- Escala necessária: **0.01** (não 0.001)
- Resultado: paredes com 33m x 28m x 3.15m

### 4. Integração do G1
O modelo do G1 utilizado:
- `g1_29dof_with_hand.xml` - versão com mãos Dex3-1
- 29 graus de liberdade + 14 nas mãos (7 por mão)
- 3 dedos por mão: thumb (3 DOF), index (2 DOF), middle (2 DOF)

### 5. Criação da Cena Unificada
```python
# Script: create_unified_scene.py
# Combina o XML do G1 com as paredes do escritório
python create_unified_scene.py
```

## 📊 Especificações Técnicas

### Robô G1
- **Altura**: ~1.2 metros
- **DOF Total**: 29 (corpo) + 14 (mãos) = 43
- **Mãos**: Dex3-1 com 3 dedos articulados
- **Massa**: ~35kg (estimado)

### Ambiente
- **Dimensões**: 33m x 28m x 3.15m
- **Origem**: Scan LiDAR de escritório real
- **Geometria**: 295 vértices, múltiplas superfícies de colisão

### Proporções
- Relação altura G1/paredes: 1:2.625 (correta)
- Escala unificada em metros

## 🛠️ Arquivos Importantes

### Essenciais:
- `g1_office_unified.xml` - Cena completa para simulação
- `3d escritorio/body1_structure/` - Meshes das paredes convertidas
- `unitree_mujoco/unitree_robots/g1/` - Modelos e meshes do G1

### Scripts de Suporte:
- `create_unified_scene.py` - Gera a cena unificada
- `extract_body1.py` - Extrai paredes do OBJ original
- `analyze_body_groups.py` - Analisa grupos no arquivo OBJ

## 📝 Notas Importantes

1. **Modelo de Mãos**: O G1 usa Dex3-1 (3 dedos), não existe versão com 5 dedos nos repositórios da Unitree
2. **Escala**: Sempre use 0.01 para converter mm→m neste projeto
3. **Posição das Paredes**: Offset de (-5.73, -7.00, 0) para centralizar

## 🎉 Resultado

A simulação está totalmente funcional com:
- ✅ Física realista
- ✅ Escalas corretas
- ✅ Robô completo e articulado
- ✅ Ambiente 3D do mundo real
- ✅ Pronto para desenvolvimento de algoritmos de controle

## 🎯 Próximas Metas

### Prioridade Alta
1. **Estabilização Inicial**: Fazer o G1 iniciar em posição de pé equilibrada
2. **Controle de Equilíbrio**: Implementar controlador básico para manter estabilidade

### Prioridade Média
3. **Caminhada**: Desenvolver algoritmo para o G1 caminhar pelo escritório
4. **Navegação**: Evitar colisões com paredes e obstáculos

### Prioridade Futura
5. **Manipulação**: Usar as mãos Dex3-1 para interagir com objetos
6. **Algoritmos Avançados**: RL/IL para comportamentos complexos

---
**Criado em**: 04/08/2024
**MuJoCo**: 3.2.7
**Unitree G1**: Modelo 29DOF com mãos Dex3-1