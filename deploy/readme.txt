deploy/
├─ site.yml
├─ hosts.ini
├─ group_vars/
│  ├─ jetson.yml
│  ├─ prod.yml
│  └─ all/
│     └─ vault.yml         
└─ roles/
   └─ wireguard/
      ├─ tasks/
      │  └─ main.yml
      ├─ files/
      │  ├─ nftables.conf
      │  └─ wireguard.nft
      └─ templates/
         └─ wg0.conf.j2

