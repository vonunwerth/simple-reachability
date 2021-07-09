import yaml

class Region:

    def __init__(self, region_yaml):
        self.initial_pose = region_yaml.get('initial_pose')
        self.count = region_yaml.get('count')
        self.poses = region_yaml.get('reachable_poses')


results = {}
with open("results.txt", 'r') as stream:
    try:
        results = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

best_pose = {}

regions = []
for region in results.get('regions'):
    regions.append(Region(region))

best_region = regions[0]
for region in regions:
    if region.count > best_region.count:
        best_region = region

print(best_region.initial_pose)
print(best_region.count)
print(best_region.poses)

