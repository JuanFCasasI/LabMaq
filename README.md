# LabMaq

## Introducción
Repositorio del laboratorio de Servo-Control del curso Dinámica de Maquinaria


## Prerequsitos
- git
- Python 3.8
- pip
- virtualenv

## Configuración del Entorno de Desarrollo


### Configuración del Entorno Python

1. **Clona el Repositorio**

    Clona este repositorio a tu máquina local usando el siguiente comando:

    ```bash
    git clone https://github.com/JuanFCasasI/LabMaq
    cd LabMaq
    ```

2. **Crea y Activa un Entorno Virtual**

    Crea un entorno virtual para manejar las dependencias del proyecto de manera aislada:

    ```bash
    virtualenv LabMaq_env --python=python3.8
    ```

    Activa el entorno virtual:

    - En Windows:
        ```bash
        ./LabMaq_env/Scripts/activate
        ```

    - En macOS y Linux:
        ```bash
        source LabMaq_env/bin/activate
        ```

3. **Instala las Dependencias**

    Con el entorno virtual activo, instala las dependencias necesarias ejecutando:

    ```bash
    pip install -r requirements.txt
    ```

## Licencia
Este proyecto está bajo la Licencia MIT - consulta el archivo `LICENSE` para más detalles.