# RECRUIT 日本橋ハーフマラソン 2025冬（AtCoder Heuristic Contest 043）
## 環境構築について
https://github.com/terry-u16/pahcer  
これを参考にしてpahcerを入れる

## pahcer run
テストケースを並列実行します。  
```
pahcer run -c sample_code
```

### オプション

- `-c`, `--comment`
  - テストケースにコメントを付与します。
  - コメントはサマリファイルなどにスコアとともに書き出されるため、解答コードの内容のメモなどにご活用ください。
- `-j`, `--json`
  - 各ケースの実行結果を表形式ではなくJSON形式でコンソールに出力します。
  - Optunaをはじめとした外部アプリケーションとの連携にご活用ください。
- `--shuffle`
  - テストケースの実行順序をシャッフルします。
  - Optunaの[WilcoxonPruner](https://tech.preferred.jp/ja/blog/wilcoxonpruner/)との連携などに使います。
- `--setting-file`
  - 読み込む設定ファイル（ `./pahcer_config.toml` ）のパスをデフォルトから変更します。
  - 一時的に実行設定を変更する場合などに使います。
- `--freeze-best-scores`
  - ベストスコアの更新を行わないようにします。
- `--no-result-file`
  - 全ケース完了後に実行結果のファイル出力を行わないようにします。
- `--no-compile`
  - 起動時にコンパイル処理を行わないようにします。