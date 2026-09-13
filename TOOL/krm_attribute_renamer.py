"""krm_attribute_renamer

現在いる階層のノードを走査し、指定したアトリビュート名を含むノードを一覧化する。
- リストの項目をクリックすると、そのノードへジャンプ（ネットワークエディタが移動＋選択）
- 「リネーム実行」で、チェックの入った項目のパラメータ文字列を一括置換

対象は「ノードのパラメータに書かれたアトリビュート名」（Wrangle の @name、
Attribute 系ノードの名前欄など）で、文字列パラメータのみを書き換える。
数値パラメータの式は触らないので、既存の挙動を壊しにくい。

配置想定:
    hsite/houdini22.0/scripts/python/krm_attribute_renamer.py
シェルフ (krm_Shelf) からの呼び出し:
    import krm_attribute_renamer, importlib
    importlib.reload(krm_attribute_renamer)
    krm_attribute_renamer.show()
"""

import re

import hou
from PySide6 import QtCore, QtWidgets


# ---------------------------------------------------------------- core logic

def _build_pattern(name):
    """アトリビュート名の単語境界マッチ用パターン。

    @P が @Pscale や i@Ptest に誤ヒットしないよう、前後が識別子文字でないことを要求する。
    先頭の @ / v@ / f@ などは識別子文字ではないのでマッチ対象に入る。
    """
    return re.compile(r"(?<![A-Za-z0-9_])" + re.escape(name) + r"(?![A-Za-z0-9_])")


def _string_parms(node):
    """文字列型パラメータだけを返す。"""
    for parm in node.parms():
        try:
            if parm.parmTemplate().type() == hou.parmTemplateType.String:
                yield parm
        except Exception:
            continue


def _preview(text, match_start):
    """該当箇所を含む1行を抜き出して表示用に整形する。"""
    line_start = text.rfind("\n", 0, match_start) + 1
    line_end = text.find("\n", match_start)
    if line_end == -1:
        line_end = len(text)
    line = text[line_start:line_end].strip()
    if len(line) > 90:
        line = line[:90] + "..."
    return line


def collect_targets(root, name, recursive):
    """root 配下のノードから name を含むものを集める。

    戻り値: [(node, [(parm, preview_text), ...]), ...]
    """
    pattern = _build_pattern(name)
    nodes = root.allSubChildren() if recursive else root.children()

    results = []
    for node in nodes:
        hits = []
        for parm in _string_parms(node):
            try:
                raw = parm.rawValue()
            except Exception:
                continue
            if not raw:
                continue
            match = pattern.search(raw)
            if match:
                hits.append((parm, _preview(raw, match.start())))
        if hits:
            results.append((node, hits))
    return results


def rename_in_parms(parms, old_name, new_name):
    """パラメータ群の文字列を置換する。書き換えた数と、スキップした理由を返す。"""
    pattern = _build_pattern(old_name)
    changed = 0
    skipped = []

    with hou.undos.group("Rename attribute: {} -> {}".format(old_name, new_name)):
        for parm in parms:
            node = parm.node()
            if node.isInsideLockedHDA():
                skipped.append("{} (ロックされたHDA内)".format(parm.path()))
                continue
            if parm.isLocked():
                skipped.append("{} (パラメータがロック)".format(parm.path()))
                continue
            try:
                raw = parm.rawValue()
                new_raw = pattern.sub(new_name, raw)
                if new_raw != raw:
                    parm.set(new_raw)
                    changed += 1
            except Exception as err:
                skipped.append("{} ({})".format(parm.path(), err))

    return changed, skipped


def jump_to(node):
    """ネットワークエディタを対象ノードの階層へ移動し、選択して画面内に収める。"""
    editor = hou.ui.paneTabOfType(hou.paneTabType.NetworkEditor)
    if editor is None:
        return
    editor.cd(node.parent().path())
    node.setCurrent(True, clear_all_selected=True)
    editor.homeToSelection()


def current_network():
    """今いる階層（ネットワークエディタの pwd）を返す。"""
    editor = hou.ui.paneTabOfType(hou.paneTabType.NetworkEditor)
    if editor is not None:
        return editor.pwd()
    return hou.node("/obj")


# ---------------------------------------------------------------------- ui

class AttributeRenamer(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super(AttributeRenamer, self).__init__(parent)
        self.setWindowTitle("Attribute Renamer")
        self.setWindowFlags(QtCore.Qt.Window)
        self.resize(680, 480)

        self._root = current_network()
        self._build_ui()
        self._update_root_label()

    # -- construction

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # 対象階層
        root_row = QtWidgets.QHBoxLayout()
        self.root_label = QtWidgets.QLabel()
        self.root_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        reload_btn = QtWidgets.QPushButton("現在の階層を取得")
        reload_btn.clicked.connect(self._on_reload_root)
        root_row.addWidget(QtWidgets.QLabel("対象:"))
        root_row.addWidget(self.root_label, 1)
        root_row.addWidget(reload_btn)
        layout.addLayout(root_row)

        # 名前入力
        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(QtCore.Qt.AlignRight)
        self.old_field = QtWidgets.QLineEdit()
        self.old_field.setPlaceholderText("例: Cd")
        self.new_field = QtWidgets.QLineEdit()
        self.new_field.setPlaceholderText("例: baseColor")
        form.addRow("変更前", self.old_field)
        form.addRow("変更後", self.new_field)
        layout.addLayout(form)

        # オプション + 検索
        option_row = QtWidgets.QHBoxLayout()
        self.recursive_check = QtWidgets.QCheckBox("サブネット内も検索")
        search_btn = QtWidgets.QPushButton("検索")
        search_btn.clicked.connect(self._on_search)
        option_row.addWidget(self.recursive_check)
        option_row.addStretch(1)
        option_row.addWidget(search_btn)
        layout.addLayout(option_row)

        self.old_field.returnPressed.connect(self._on_search)

        # 結果ツリー
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["ノード / パラメータ", "該当箇所"])
        self.tree.setColumnWidth(0, 280)
        self.tree.setAlternatingRowColors(True)
        self.tree.setRootIsDecorated(True)
        self.tree.itemClicked.connect(self._on_item_clicked)
        layout.addWidget(self.tree, 1)

        # フッタ
        footer = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("アトリビュート名を入力して検索してください。")
        check_all_btn = QtWidgets.QPushButton("全選択")
        check_all_btn.clicked.connect(lambda: self._set_all_checked(True))
        uncheck_all_btn = QtWidgets.QPushButton("全解除")
        uncheck_all_btn.clicked.connect(lambda: self._set_all_checked(False))
        self.rename_btn = QtWidgets.QPushButton("リネーム実行")
        self.rename_btn.setEnabled(False)
        self.rename_btn.clicked.connect(self._on_rename)
        footer.addWidget(self.status_label, 1)
        footer.addWidget(check_all_btn)
        footer.addWidget(uncheck_all_btn)
        footer.addWidget(self.rename_btn)
        layout.addLayout(footer)

    # -- helpers

    def _update_root_label(self):
        self.root_label.setText(self._root.path() if self._root else "(なし)")

    def _iter_node_items(self):
        for i in range(self.tree.topLevelItemCount()):
            yield self.tree.topLevelItem(i)

    def _set_all_checked(self, checked):
        state = QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked
        for item in self._iter_node_items():
            item.setCheckState(0, state)

    # -- slots

    def _on_reload_root(self):
        self._root = current_network()
        self._update_root_label()

    def _on_search(self):
        old_name = self.old_field.text().strip()
        if not old_name:
            self.status_label.setText("変更前のアトリビュート名を入力してください。")
            return
        if self._root is None:
            self.status_label.setText("対象の階層が取得できませんでした。")
            return

        self.tree.clear()
        results = collect_targets(self._root, old_name, self.recursive_check.isChecked())

        parm_count = 0
        for node, hits in results:
            node_item = QtWidgets.QTreeWidgetItem(self.tree)
            node_item.setText(0, "{}  [{}]".format(node.name(), node.type().name()))
            node_item.setText(1, node.path())
            node_item.setCheckState(0, QtCore.Qt.Checked)
            node_item.setData(0, QtCore.Qt.UserRole, node.path())
            node_item.setData(0, QtCore.Qt.UserRole + 1, [p.name() for p, _ in hits])
            node_item.setExpanded(True)

            for parm, preview in hits:
                parm_item = QtWidgets.QTreeWidgetItem(node_item)
                parm_item.setText(0, parm.name())
                parm_item.setText(1, preview)
                parm_item.setData(0, QtCore.Qt.UserRole, node.path())
                parm_count += 1

        found = self.tree.topLevelItemCount()
        self.rename_btn.setEnabled(found > 0)
        if found:
            self.status_label.setText(
                "{} ノード / {} パラメータが該当（項目クリックでジャンプ）".format(found, parm_count)
            )
        else:
            self.status_label.setText("該当するノードはありませんでした。")

    def _on_item_clicked(self, item, column):
        path = item.data(0, QtCore.Qt.UserRole)
        node = hou.node(path) if path else None
        if node:
            jump_to(node)

    def _on_rename(self):
        old_name = self.old_field.text().strip()
        new_name = self.new_field.text().strip()
        if not old_name or not new_name:
            self.status_label.setText("変更前と変更後の両方を入力してください。")
            return
        if old_name == new_name:
            self.status_label.setText("変更前と変更後が同じです。")
            return

        parms = []
        for item in self._iter_node_items():
            if item.checkState(0) != QtCore.Qt.Checked:
                continue
            node = hou.node(item.data(0, QtCore.Qt.UserRole))
            if node is None:
                continue
            for parm_name in item.data(0, QtCore.Qt.UserRole + 1):
                parm = node.parm(parm_name)
                if parm is not None:
                    parms.append(parm)

        if not parms:
            self.status_label.setText("チェックされた項目がありません。")
            return

        changed, skipped = rename_in_parms(parms, old_name, new_name)

        message = "{} パラメータを {} → {} に置換しました。".format(changed, old_name, new_name)
        if skipped:
            message += " スキップ {} 件。".format(len(skipped))
            print("[Attribute Renamer] スキップした項目:")
            for line in skipped:
                print("  " + line)
        self.status_label.setText(message)

        self._on_search()


_window = None


def show():
    global _window
    if _window is not None:
        _window.close()
    _window = AttributeRenamer(parent=hou.qt.mainWindow())
    _window.show()
    return _window
