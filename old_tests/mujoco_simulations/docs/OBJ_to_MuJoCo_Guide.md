# Guia de Conversão: OBJ do Escritório para MuJoCo

## Visão Geral do Processo

Este guia documenta o processo completo para converter o arquivo `escritorio_CW_scan.obj` (scan 3D do escritório) em componentes funcionais para simulação no MuJoCo com o robô Unitree G1.

## Análise do Arquivo OBJ

### Estrutura Identificada
- **Total de vértices**: 59,858
- **Total de faces**: 120,128
- **Grupos (g)**: 131 grupos únicos
- **Materiais**: 3 (Steel_-_Satin, Oak, MATERIAL_NOT_DEFINED)
- **Arquivo MTL**: f8fe65f8-8d7b-4bbf-a14c-457c60f2b849.mtl

### Principais Componentes Identificados

#### 1. Paredes
- `mesa parede tvs` (650 faces) - única parede identificada pelo nome

#### 2. Móveis (maiores grupos por número de faces)
- Cadeiras giratórias (5,450 faces cada):
  - `cadeira giratoria`
  - `cadeira_giratoria_tvs`
  - `cadeira_giratoria_tvs_2`
  - `cadeira-giratoria_2`
  - `cadeira_giratoria_3`
- Sofás (4,088 faces cada):
  - `sofa_1`
  - `sofa_3`
- Múltiplos componentes `Body3:*` e `Body7:*` (~3,000+ faces cada)

## Instalação de Dependências

Execute os seguintes comandos como sudo em outro terminal:

```bash
# 1. Instalar Python e ferramentas essenciais
sudo apt update
sudo apt install -y python3-pip python3-venv python3-full

# 2. Criar ambiente virtual (recomendado)
python3 -m venv ~/mujoco_env
source ~/mujoco_env/bin/activate

# 3. Instalar pacotes Python necessários
pip install trimesh
pip install obj2mjcf
pip install coacd
pip install numpy Pillow lxml
pip install termcolor tyro

# 4. Instalar dependências para visualização (opcional)
pip install pyglet matplotlib
```

## Processo de Conversão

### Fase 1: Preparação e Extração de Paredes

#### 1.1 Script de Extração de Grupos Específicos

```python
#!/usr/bin/env python3
"""
extract_walls.py - Extrai grupos de paredes do OBJ
"""

import trimesh
import numpy as np
from pathlib import Path

def extract_wall_groups(obj_path, output_dir):
    """Extrai grupos que parecem ser paredes"""
    
    # Criar diretório de saída
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Carregar arquivo OBJ
    print(f"Carregando {obj_path}...")
    scene = trimesh.load(
        obj_path,
        split_object=True,
        group_material=True,
        process=False
    )
    
    if isinstance(scene, trimesh.Scene):
        print(f"Cena carregada com {len(scene.geometry)} geometrias")
        
        # Identificar possíveis paredes
        wall_candidates = []
        
        for name, geom in scene.geometry.items():
            bounds = geom.bounds
            size = bounds[1] - bounds[0]
            
            # Critérios para identificar paredes:
            # 1. Nome contém "wall" ou "parede"
            # 2. Altura > 2m e espessura < 0.5m
            is_wall_by_name = any(word in name.lower() for word in ['wall', 'parede', 'muro'])
            is_wall_by_shape = size[2] > 2.0 and min(size[0], size[1]) < 0.5
            
            if is_wall_by_name or is_wall_by_shape:
                wall_candidates.append((name, geom, size))
                print(f"  Parede candidata: {name}")
                print(f"    Dimensões: X={size[0]:.2f}m, Y={size[1]:.2f}m, Z={size[2]:.2f}m")
        
        # Exportar paredes individuais
        for name, geom, size in wall_candidates:
            safe_name = name.replace(' ', '_').replace(':', '_')
            output_file = output_dir / f"wall_{safe_name}.obj"
            geom.export(output_file.as_posix())
            print(f"  Exportado: {output_file}")
            
        # Criar arquivo combinado de todas as paredes
        if wall_candidates:
            combined = trimesh.util.concatenate([wc[1] for wc in wall_candidates])
            combined.export((output_dir / "all_walls_combined.obj").as_posix())
            print(f"  Todas as paredes combinadas: all_walls_combined.obj")
    
    return wall_candidates

if __name__ == "__main__":
    extract_wall_groups(
        "/home/pedro_setubal/Workspaces/G1/3d escritorio/escritorio_CW_scan.obj",
        "/home/pedro_setubal/Workspaces/G1/walls_extracted"
    )
```

### Fase 2: Conversão com obj2mjcf

#### 2.1 Conversão Básica

```bash
# Criar diretório para conversão
mkdir -p ~/Workspaces/G1/mjcf_conversion

# Copiar arquivos necessários
cp "3d escritorio/escritorio_CW_scan.obj" mjcf_conversion/
cp "3d escritorio/f8fe65f8-8d7b-4bbf-a14c-457c60f2b849.mtl" mjcf_conversion/

# Converter com obj2mjcf
cd mjcf_conversion
obj2mjcf --obj-dir . \
         --save-mjcf \
         --decompose \
         --overwrite \
         --verbose
```

#### 2.2 Script de Conversão Customizado para Paredes

```python
#!/usr/bin/env python3
"""
convert_walls_to_mjcf.py - Converte paredes para MJCF com configurações específicas
"""

import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

def convert_wall_to_mjcf(wall_obj_path, scale=1.0):
    """Converte uma parede OBJ para MJCF"""
    
    wall_path = Path(wall_obj_path)
    output_dir = wall_path.parent / wall_path.stem
    
    # Executar obj2mjcf
    cmd = [
        "obj2mjcf",
        "--obj-dir", str(wall_path.parent),
        "--obj-filter", wall_path.name,
        "--save-mjcf",
        "--compile-model",
        "--verbose",
        "--overwrite"
    ]
    
    print(f"Convertendo {wall_path.name}...")
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ Conversão bem-sucedida!")
        
        # Modificar MJCF gerado para adicionar propriedades de colisão
        mjcf_file = output_dir / f"{wall_path.stem}.xml"
        if mjcf_file.exists():
            customize_wall_mjcf(mjcf_file, scale)
    else:
        print(f"❌ Erro na conversão: {result.stderr}")
        
def customize_wall_mjcf(mjcf_path, scale=1.0):
    """Customiza MJCF da parede para simulação"""
    
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    
    # Adicionar configurações de colisão padrão
    worldbody = root.find('worldbody')
    if worldbody is not None:
        # Encontrar todos os geoms
        for geom in worldbody.findall('.//geom'):
            # Configurar propriedades de colisão
            geom.set('contype', '1')
            geom.set('conaffinity', '1')
            geom.set('friction', '1 0.1 0.1')  # Atrito realista para parede
            geom.set('rgba', '0.9 0.9 0.9 1')  # Cor cinza claro
            
            # Aplicar escala se necessário
            if scale != 1.0:
                if 'mesh' in geom.attrib:
                    geom.set('scale', f'{scale} {scale} {scale}')
    
    # Salvar modificações
    tree.write(mjcf_path)
    print(f"  ✏️ MJCF customizado: {mjcf_path}")

if __name__ == "__main__":
    # Converter paredes extraídas
    walls_dir = Path("/home/pedro_setubal/Workspaces/G1/walls_extracted")
    for wall_obj in walls_dir.glob("wall_*.obj"):
        convert_wall_to_mjcf(wall_obj)
```

### Fase 3: Integração no Simulador

#### 3.1 Criar Cena de Teste com Paredes

```xml
<!-- test_walls_scene.xml -->
<mujoco model="g1_walls_test">
  <!-- Incluir modelo do G1 -->
  <include file="../unitree_robots/g1/g1_29dof_with_hand.xml"/>
  
  <!-- Incluir paredes convertidas -->
  <include file="walls_extracted/wall_mesa_parede_tvs/wall_mesa_parede_tvs.xml"/>
  
  <statistic center="0 0 0.5" extent="5.0"/>
  
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>
  
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    
    <!-- Chão -->
    <geom name="floor" size="10 10 0.01" type="plane" rgba="0.5 0.5 0.5 1"/>
    
    <!-- Posicionar paredes manualmente se necessário -->
    <body name="walls_group" pos="0 0 0">
      <!-- Paredes serão incluídas aqui via include -->
    </body>
  </worldbody>
</mujoco>
```

#### 3.2 Script de Validação

```python
#!/usr/bin/env python3
"""
validate_walls.py - Valida paredes no simulador MuJoCo
"""

import mujoco
import numpy as np

def validate_wall_collision(mjcf_path):
    """Testa colisão de paredes no MuJoCo"""
    
    try:
        # Carregar modelo
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)
        
        print(f"✅ Modelo carregado com sucesso!")
        print(f"  Corpos: {model.nbody}")
        print(f"  Geoms: {model.ngeom}")
        
        # Verificar geoms de parede
        for i in range(model.ngeom):
            name = model.geom(i).name
            if 'wall' in name.lower() or 'parede' in name.lower():
                print(f"  Parede encontrada: {name}")
                
        # Simular alguns passos
        for _ in range(100):
            mujoco.mj_step(model, data)
            
        print("✅ Simulação executada sem erros!")
        
    except Exception as e:
        print(f"❌ Erro: {e}")

if __name__ == "__main__":
    validate_wall_collision("test_walls_scene.xml")
```

## Processo Incremental Recomendado

### 1. **Primeira Iteração - Parede Única**
   - Extrair apenas `mesa parede tvs`
   - Converter com obj2mjcf
   - Testar no simulador com G1
   - Ajustar escala e posição

### 2. **Segunda Iteração - Estrutura Principal**
   - Identificar e extrair elementos estruturais (Body3:*, Body7:*)
   - Analisar se são paredes, pilares ou divisórias
   - Converter e posicionar no simulador

### 3. **Terceira Iteração - Móveis Grandes**
   - Converter sofás e mesas grandes
   - Configurar como objetos estáticos
   - Testar colisão com robô

### 4. **Iteração Final - Objetos Pequenos**
   - Adicionar cadeiras e objetos menores
   - Configurar alguns como dinâmicos (móveis)
   - Otimizar performance

## Comandos Úteis

```bash
# Visualizar estrutura do OBJ
grep -E "^(g|o|usemtl)" escritorio_CW_scan.obj | head -50

# Contar elementos por grupo
grep "^g " escritorio_CW_scan.obj | sort | uniq -c | sort -nr

# Extrair grupo específico manualmente
awk '/^g mesa parede tvs/,/^g [^m]/' escritorio_CW_scan.obj > wall_mesa_parede_tvs.obj

# Testar conversão rápida
obj2mjcf --obj-dir . --obj-filter "wall_*.obj" --save-mjcf --verbose
```

## Problemas Comuns e Soluções

### 1. **Escala Incorreta**
   - Verificar unidades no OBJ (metros vs milímetros)
   - Usar parâmetro `scale` no MJCF: `<mesh scale="0.001 0.001 0.001"/>`

### 2. **Orientação Errada**
   - Adicionar rotação no body: `<body euler="1.57 0 0">`
   - Verificar sistema de coordenadas (Y-up vs Z-up)

### 3. **Colisões Não Funcionam**
   - Verificar `contype` e `conaffinity`
   - Garantir que meshes são fechadas (watertight)
   - Considerar usar decomposição convexa

### 4. **Performance Ruim**
   - Reduzir complexidade das meshes
   - Usar primitivas geométricas quando possível
   - Combinar objetos estáticos

## Próximos Passos

1. Instalar dependências listadas
2. Executar script de extração de paredes
3. Converter primeira parede com obj2mjcf
4. Integrar no simulador e validar
5. Proceder incrementalmente com outros objetos