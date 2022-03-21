import xml.etree.ElementTree as ET

tree = ET.parse('model.sdf')
root = tree.getroot()

mass = root.find('model').find('link').find('inertial').find('mass')
mass.text = '555'

print(mass.text)

for link in root.find('model').findall('link'):
    print(link.find('pose').text)

# coin = root.get('coin')
# print(f"Crypto name = {coin}")

tree.write('output.sdf')