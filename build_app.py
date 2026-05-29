import vitis

client = vitis.create_client()
client.set_workspace(path="C:/Users/kimse/capstone/antidrone/vitis_workspace")

app = client.get_component(name="antidrone_app")
status = app.build()
print("Build status:", status)

vitis.dispose()
