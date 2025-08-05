# Status do Projeto - Unitree G1 MuJoCo Simulation

**Última atualização**: 04/08/2024

## 🎯 Objetivo
Criar simulação completa do Unitree G1 (com mãos de 5 dedos) em ambiente de escritório escaneado via LiDAR.

## ✅ Concluído

### 1. Instalação MuJoCo 3.2.7
```bash
# Testado e funcionando:
simulate  # Abre janela do simulador
```

### 2. Teste inicial do G1
- Modelo `scene_29dof_with_hand.xml` carrega corretamente
- Física e gravidade funcionando
- **PROBLEMA**: Mãos mostram apenas 3 dedos (não 5 como desejado)

### 3. Análise do arquivo OBJ
- 131 grupos identificados
- 59,858 vértices, 120,128 faces
- Apenas 1 parede identificada: "mesa parede tvs"
- Múltiplos móveis: cadeiras, sofás, componentes Body*

### 4. Investigação modelo G1 com 5 dedos
- **CONCLUSÃO**: O modelo disponível usa Dex3-1 (3 dedos por mão)
- Dedos disponíveis: thumb (3 juntas), index (2 juntas), middle (2 juntas)
- Total: 14 DOF nas mãos (7 por mão)
- Não foi encontrado modelo com 5 dedos nos repositórios da Unitree

### 5. Descoberta e Conversão das Paredes do Escritório
- **PROCESSO**: 
  1. Primeira tentativa: grupo "mesa parede tvs" - era apenas um móvel de 10cm altura
  2. Análise sistemática: busca por grupos "Body" no arquivo OBJ
  3. **DESCOBERTA**: Body1 contém as paredes reais!
     - Dimensões originais: 3283mm x 2836mm x 315mm
     - Dimensões reais: **33m x 28m x 3.15m** (escritório grande)
     - 295 vértices, estrutura complexa
  4. Conversão bem-sucedida com obj2mjcf
  5. **ESCALA CORRETA**: 0.01 (10x maior que inicial)
     - Arquivo OBJ está em milímetros
     - Scale 0.01 converte para metros corretamente
- **ARQUIVO VALIDADO**: g1_office_scale_fixed.xml com escala correta

## ✅ Concluído (cont.)

### 6. Validação de Escala e Visualização
- **Paredes**: 33m x 28m x 3.15m confirmadas
- **G1**: ~1.2m altura (proporção correta 1:2.6)
- **Arquivos de teste**:
  - `g1_office_working.xml` - apenas paredes
  - `scene_29dof_with_hand.xml` - G1 completo
  - `compare_scale.sh` - script para comparação

## 🚧 Em Progresso

### 1. Criar Cena Unificada G1 + Escritório
- Problema: include do G1 não funciona diretamente
- Solução: copiar estrutura completa do G1 para arquivo único
- Alternativa: usar unitree_mujoco compilado

### 2. SDK e Compilação
- unitree_sdk2: instalação parcial (falta eigen3)
- unitree_mujoco: não compila sem SDK completo
- Workaround atual: usar MuJoCo direto

## 📋 Próximos Passos

### Imediatos:
1. **Investigar modelos G1**: Procurar versão com 5 dedos
2. **Instalar dependências Python** (em venv):
   ```bash
   python3 -m venv ~/mujoco_env
   source ~/mujoco_env/bin/activate
   pip install trimesh obj2mjcf coacd
   ```

### Conversão do Ambiente:
1. Extrair grupo "mesa parede tvs" do OBJ
2. Converter para MJCF usando obj2mjcf
3. Testar escala e colisões
4. Adicionar incrementalmente outros objetos

## 🛠️ Comandos Úteis

```bash
# Testar G1 atual:
cd ~/Workspaces/G1
./test_g1_direct.sh

# Analisar OBJ:
python3 analyze_obj_simple.py

# Ver estrutura do G1:
ls unitree_mujoco/unitree_robots/g1/
```

## 📝 Arquivos Importantes

- `CLAUDE.md` - Contexto para Claude Code
- `README.md` - Documentação completa
- `OBJ_to_MuJoCo_Guide.md` - Guia de conversão 3D
- `test_g1_direct.sh` - Script de teste rápido

## ⚠️ Problemas Conhecidos

1. **Mãos 3 vs 5 dedos** - Modelo atual não é o desejado
2. **SDK incompleto** - Falta eigen3 para dex3
3. **Python sistema** - Requer venv para pip