# MuJoCo Simulation Archive

Este diretório contém todos os arquivos relacionados às simulações do robô Unitree G1 no **MuJoCo**, desenvolvidas na primeira fase do projeto.

## ✅ Status: CONCLUÍDO COM SUCESSO

A simulação MuJoCo foi finalizada com êxito, incluindo:
- G1 humanoid com mãos de 5 dedos funcionando
- Ambiente 3D do escritório integrado
- Física realista e controles funcionais

## 📁 Estrutura de Arquivos

### `/mujoco_simulations/`
- **`xml_files/`**: Arquivos XML de simulação do MuJoCo
  - `g1_5fingers_*.xml` - Simulações com mãos de 5 dedos
  - `g1_office_*.xml` - Simulações com ambiente de escritório
  - `*_unified.xml` - Versões finais integradas

- **`scripts/`**: Scripts Python e shell utilizados
  - `create_unified_scene.py` - Integração robô + ambiente
  - `extract_body1.py` - Extração das paredes do OBJ
  - `analyze_obj_simple.py` - Análise do arquivo 3D

- **`launchers/`**: Scripts de execução
  - `launch_g1_5fingers_final.sh` - Launcher principal
  - `launch_g1_5fingers_with_office.sh` - Menu interativo
  - `launch_office_with_g1.sh` - Ambiente + robô

- **`docs/`**: Documentação técnica
  - `OBJ_to_MuJoCo_Guide.md` - Guia de conversão 3D
  - `MUJOCO_LOG.TXT` - Logs de desenvolvimento

### `/mujoco_assets/`
- **`meshes/`**: Arquivos STL dos componentes do G1
- **`g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`**: Modelo URDF original

### `/mujoco_source/`
- Código fonte completo do MuJoCo 3.2.7

### `/unitree_mujoco_integration/`
- Integração oficial Unitree-MuJoCo

## 🎯 Principais Conquistas

1. **Modelo G1 Completo**: 29 DOF + mãos RH56DFQ de 5 dedos
2. **Ambiente Realista**: Escritório 33m x 28m x 3.15m escaneado via LiDAR
3. **Escala Correta**: Robô 1.2m altura vs paredes 3.15m
4. **Física Funcional**: Gravidade, colisões, controles responsivos
5. **Interface Amigável**: Launchers automatizados e menu interativo

## 🚀 Como Executar (Referência)

**Para executar as simulações MuJoCo arquivadas:**

```bash
# G1 com mãos de 5 dedos
cd old_tests/mujoco_simulations/launchers
./launch_g1_5fingers_final.sh

# Ou comando direto:
cd ~/Workspaces/G1/unitree_ros/robots/g1_description
simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
```

## ⚠️ Status do Projeto

**Esta fase está FINALIZADA.** O desenvolvimento continua com **Gazebo ROS2** para:
- Integração com ROS2 Jazzy
- Controle avançado do robô
- Sensores e atuadores realistas
- Desenvolvimento de algoritmos de controle

---

*Arquivado em: $(date '+%d/%m/%Y')*  
*Próxima fase: Simulação Gazebo ROS2*