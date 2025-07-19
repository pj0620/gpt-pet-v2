import os
import xacro

def main():
    src = os.path.join(os.path.dirname(__file__), 'urdf', 'gptpet.xacro')
    dst_dir = os.path.join(os.environ['AMENT_PREFIX_PATH'].split(':')[0], 'share', 'startup', 'urdf')
    os.makedirs(dst_dir, exist_ok=True)
    dst = os.path.join(dst_dir, 'gptpet.urdf')
    doc = xacro.process_file(src)
    with open(dst, 'w') as f:
        f.write(doc.toprettyxml())

if __name__ == '__main__':
    main()
