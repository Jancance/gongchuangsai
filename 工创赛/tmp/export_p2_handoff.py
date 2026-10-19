import base64, hashlib, json, pathlib, re

root = pathlib.Path(r'D:\Codex\工创赛')
source = pathlib.Path(r'D:\Codex\home\sessions\2026\08\29\rollout-2026-08-29T16-40-10-01a04cad-23b6-7d81-8bd6-b5076a65d1fe.jsonl')
dest = root / 'evidence/p2_handoff_2026-09-13'
dest.mkdir(parents=True, exist_ok=True)
lines = ['# 原“视觉”任务可读对话存档', '', '来源任务：01a04cad-23b6-7d81-8bd6-b5076a65d1fe；导出日期：2026-09-13。', '', '按原始顺序保留用户和助手可见消息；不复制内部推理、工具调用及原始会话整包。历史判断和网络地址可能过时，接续以 P2_MODEL_HANDOFF_2026-09-13.md 为准。图片按内容去重保存；外部附件的可用情况见清单。', '']
assets, missing, count = {}, set(), 0
def save_asset(data, suffix, origin):
    digest = hashlib.sha256(data).hexdigest()
    name = digest[:16] + suffix
    target = dest / name
    target.write_bytes(data)
    assert hashlib.sha256(target.read_bytes()).hexdigest() == digest
    assets[origin] = {'file': str(target.relative_to(root)), 'sha256': digest, 'bytes': len(data)}
    return '../' + str(target.relative_to(root)).replace('\\', '/')
for raw in source.open(encoding='utf-8'):
    event = json.loads(raw)
    p = event.get('payload', {})
    if event['type'] != 'response_item' or p.get('type') != 'message' or p.get('role') not in ('user', 'assistant'):
        continue
    count += 1
    lines.extend([f"## {count}. {p['role']} | {event.get('timestamp', '')}", ''])
    for c in p.get('content', []):
        if c.get('text'):
            text = c['text']
            lines.extend([text, ''])
            for path in re.findall(r'[CD]:[\\/][^\r\n]+?\.(?:jpg|jpeg|png|txt)(?=[\s\"\)]|$)', text, re.I):
                path = path.replace('\\\\', '\\')
                f = pathlib.Path(path)
                if f.is_file():
                    link = save_asset(f.read_bytes(), f.suffix, path)
                    lines.extend([f'已接续附件：[{f.name}]({link})', ''])
                else:
                    missing.add(path)
        url = c.get('image_url', '')
        if isinstance(url, dict):
            url = url.get('url', '')
        if isinstance(url, str) and url.startswith('data:image/'):
            header, encoded = url.split(',', 1)
            suffix = '.jpg' if 'jpeg' in header else '.png' if 'png' in header else '.webp'
            link = save_asset(base64.b64decode(encoded), suffix, f'embedded-message-{count}')
            lines.extend([f'![原对话图片]({link})', ''])
archive = root / 'docs/P2_CONVERSATION_ARCHIVE_2026-09-13.md'
archive.write_text('\n'.join(lines), encoding='utf-8')
manifest = {'source_thread': '01a04cad-23b6-7d81-8bd6-b5076a65d1fe', 'source_bytes': source.stat().st_size, 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'message_count': count, 'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(), 'assets': assets, 'missing_external_paths': sorted(missing)}
(dest / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({'messages': count, 'asset_references': len(assets), 'missing_external_paths': sorted(missing), 'archive_bytes': archive.stat().st_size, 'evidence_bytes': sum(f.stat().st_size for f in dest.iterdir()), 'source_bytes': source.stat().st_size}, ensure_ascii=False))
