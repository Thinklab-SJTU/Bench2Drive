# Check if carla is working properly

### Verification Process
1. Run in terminal 1
    ```bash
        ./CarlaUE4.sh -RenderOffScreen -nosound -fps=10 -carla-rpc-port=2000
    ```
2. Run in terninal 2
    ```python
        import carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(100.0)
        list_map = client.get_available_maps()
        client_version = client.get_client_version()
        server_version = client.get_server_version()
        world = client.get_world()
        world = client.load_world('Town02')
    ```
3. If executed successfully, carla runs successfully.

### Summary of some issues
- The normal startup of carla is **blocking**. 

- If the process ends immediately, please check **vulkaninfo**
    - if vulkaninfo does not exist
        ```bash
            apt install vulkan-tools # 	vulkaninfo
	        apt install libgeos-dev
        ``` 
    - if **test only** appears, please check Ubuntu system version
        - A100/H100/A800/H800 requires a newer version of vulkaninfo, and therefore requires Ubuntu 22.04
