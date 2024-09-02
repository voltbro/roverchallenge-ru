# Информация о разработке

Настройкка deploy-ключей в github:

- `ssh-keygen` - save to `./repo.key`
- `git config core.sshCommand "ssh -i $(pwd)/repo.key -F /dev/null"`

## Зависимости для разработки

**APT**:

- `just` ([нужен доп. репозиторий](https://github.com/casey/just?tab=readme-ov-file#packages))
